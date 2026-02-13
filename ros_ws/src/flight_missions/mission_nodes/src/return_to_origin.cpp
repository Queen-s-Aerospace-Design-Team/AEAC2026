#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <stdint.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace
{
    constexpr float PI                   = 3.14159f;
    constexpr float POSITION_THRESHOLD   = 0.25f;                     // meters
    constexpr float VELOCITY_THRESHOLD   = 0.30f;                     // m/s
    constexpr std::chrono::seconds DWELL = std::chrono::seconds( 1 ); // must stay within band for this long

    // Origin target
    constexpr float TARGET_X = 0.0f;
    constexpr float TARGET_Y = 0.0f;
    constexpr float TARGET_Z = -2.5f;
} // namespace

namespace Utilities
{
    static bool waitForServices( rclcpp::Node* node, const std::vector<rclcpp::ClientBase::SharedPtr>& services,
                                 std::chrono::seconds timeout = std::chrono::seconds( 1 ) )
    {
        for( const auto& service : services )
        {
            RCLCPP_INFO_STREAM( node->get_logger(), "Waiting for service: " << service->get_service_name() );

            while( !service->wait_for_service( timeout ) )
            {
                if( !rclcpp::ok() )
                {
                    RCLCPP_ERROR( node->get_logger(), "Interrupted while waiting for service: %s", service->get_service_name() );
                    return false;
                }

                RCLCPP_WARN( node->get_logger(), "Service %s not available, waiting again...", service->get_service_name() );
            }
        }

        return true;
    }
} // namespace Utilities

class ReturnToOrigin : public rclcpp::Node
{
  public:
    ReturnToOrigin()
        : Node( "return_to_origin" )
        , m_offboardControlMode_pub( create_publisher<OffboardControlMode>( "/fmu/in/offboard_control_mode", 10 ) )
        , m_trajectorySetpoint_pub( create_publisher<TrajectorySetpoint>( "/fmu/in/trajectory_setpoint", 10 ) )
        , m_vehicleCommand_pub( create_publisher<VehicleCommand>( "/fmu/in/vehicle_command", 10 ) )
        , m_vehicleCommand_client( create_client<px4_msgs::srv::VehicleCommand>( "/fmu/vehicle_command" ) )
        , m_state( FSM::Init )
        , m_inReachedBand( false )
        , m_serviceDone( false )
        , m_serviceResult( 0 )
    {
        RCLCPP_INFO( this->get_logger(), "Starting test mission: %s", get_name() );

        if( !Utilities::waitForServices( this, { m_vehicleCommand_client } ) )
        {
            RCLCPP_ERROR( get_logger(), "Failed to validate services. Exiting..." );
            return;
        }

        rclcpp::QoS bestEffortQoS = rclcpp::QoS( 10 ).best_effort();
        m_localPosition_sub       = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", bestEffortQoS,
            std::bind( &ReturnToOrigin::onVehicleLocalPosition, this, std::placeholders::_1 ) );

        // Main loop
        auto state_timer_callback = [this]() -> void
        {
            // If we are not done, continue sending setpoints and offboard control
            if( m_state < FSM::ManualRequested )
            {
                // offboard_control_mode needs to be paired with trajectory_setpoint
                publishOffboardControlMode();
                publishTrajectorySetpoint();
            }

            switch( m_state )
            {
                case Init:
                {
                    requestOffboardControlMode();
                    m_state = OffboardRequested;
                    break;
                }
                case OffboardRequested:
                {
                    if( m_serviceDone )
                    {
                        if( m_serviceResult == 0 )
                        {
                            RCLCPP_INFO( this->get_logger(), "Entered offboard mode" );
                            m_state = FSM::WaitForStableOffboard;
                        }
                        else
                        {
                            RCLCPP_ERROR( this->get_logger(), "Failed to enter offboard mode, exiting" );
                            rclcpp::shutdown();
                        }
                    }

                    break;
                }
                case WaitForStableOffboard:
                {
                    static uint8_t offboardCounter = 0;

                    if( ++offboardCounter > 10 )
                    {
                        arm();
                        m_state = FSM::ArmRequested;
                    }

                    break;
                }
                case ArmRequested:
                {
                    if( m_serviceDone )
                    {
                        if( m_serviceResult == 0 )
                        {
                            RCLCPP_INFO( this->get_logger(), "UAV armmed" );
                            m_state = FSM::Approach;
                        }
                        else
                        {
                            RCLCPP_ERROR( this->get_logger(), "Failed to arm. Exiting..." );
                            rclcpp::shutdown();
                        }
                    }

                    break;
                }
                case Approach:
                {
                    static rclcpp::Time reachedSince;

                    const float dx    = m_localPosition->x - TARGET_X;
                    const float dy    = m_localPosition->y - TARGET_Y;
                    const float dz    = m_localPosition->z - TARGET_Z;
                    const float dist  = std::sqrt( dx * dx + dy * dy + dz * dz );
                    const bool inBand = dist < POSITION_THRESHOLD;

                    RCLCPP_INFO( get_logger(), "UAV is %f meters to target.", dist );

                    if( inBand )
                    {
                        if( !m_inReachedBand )
                        {
                            m_inReachedBand = true;
                            reachedSince    = now();
                        }
                        else if( ( now() - reachedSince ) > rclcpp::Duration( DWELL ) )
                        {
                            RCLCPP_INFO( this->get_logger(),
                                         "Reached setpoint -> mission accomplished -> requesting manual control mode..." );
                            requestManualControlMode();
                            m_state = FSM::ManualRequested;
                        }
                    }
                    else
                    {
                        m_inReachedBand = false;
                    }

                    break;
                }
                case ManualRequested:
                {
                    if( m_serviceDone )
                    {
                        if( m_serviceResult == 0 )
                        {
                            RCLCPP_INFO( this->get_logger(), "Manual mode resumed" );
                            m_state = FSM::Finished;
                        }
                        else
                        {
                            RCLCPP_ERROR( this->get_logger(), "Failed to disengage offboard control mode. Exiting..." );
                            rclcpp::shutdown();
                        }
                    }

                    break;
                }
                case Finished:
                {
                    m_timer->cancel(); // stop this timer callback
                    RCLCPP_INFO( this->get_logger(), "Mission Completed: Mission timer callback stopped." );
                    break;
                }
            }
        };

        m_timer = create_wall_timer( 100ms, state_timer_callback );
    }

    ~ReturnToOrigin() override { RCLCPP_INFO( this->get_logger(), "%s destructor called.", this->get_name() ); }

  private:
    enum FSM : uint8_t
    {
        Init,
        OffboardRequested,
        WaitForStableOffboard,
        ArmRequested,
        Approach,
        ManualRequested,
        Finished
    };

    void requestManualControlMode();
    void requestOffboardControlMode();
    void arm();
    void disarm();
    void publishOffboardControlMode();
    void publishTrajectorySetpoint();
    void requestVehicleCommand( uint16_t command, float param1 = 0.0, float param2 = 0.0 );
    void onVehicleLocalPosition( const VehicleLocalPosition::SharedPtr msg );
    void responseCallback( rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future );

    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr m_offboardControlMode_pub;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr m_trajectorySetpoint_pub;
    rclcpp::Publisher<VehicleCommand>::SharedPtr m_vehicleCommand_pub;

    // Subscriptions
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr m_localPosition_sub;

    // Clients
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr m_vehicleCommand_client;

    VehicleLocalPosition::SharedPtr m_localPosition;

    FSM m_state;
    bool m_inReachedBand;
    bool m_serviceDone;
    uint8_t m_serviceResult;

    rclcpp::TimerBase::SharedPtr m_timer;
};

void ReturnToOrigin::requestOffboardControlMode()
{
    RCLCPP_INFO( this->get_logger(), "Requesting switch to Offboard mode..." );
    requestVehicleCommand( VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6 );
}

void ReturnToOrigin::requestManualControlMode()
{
    RCLCPP_INFO( this->get_logger(), "Requesting switch to Offboard mode..." );
    requestVehicleCommand( VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3 );
}

void ReturnToOrigin::arm()
{
    RCLCPP_INFO( this->get_logger(), "Arm command send..." );
    requestVehicleCommand( VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0 );
}

void ReturnToOrigin::disarm()
{
    RCLCPP_INFO( this->get_logger(), "Disarm command send..." );
    requestVehicleCommand( VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0 );
}

void ReturnToOrigin::publishOffboardControlMode()
{
    OffboardControlMode msg{};
    msg.position     = true;
    msg.velocity     = false;
    msg.acceleration = false;
    msg.attitude     = false;
    msg.body_rate    = false;
    msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
    m_offboardControlMode_pub->publish( msg );
}

void ReturnToOrigin::publishTrajectorySetpoint()
{
    TrajectorySetpoint msg{};
    msg.position = { TARGET_X, TARGET_Y, TARGET_Z };
    // msg.velocity  = { 0 };              // { 5.0f, 5.0f, -1.0f };
    msg.yaw       = -1 * PI; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // msg.yawspeed  = ( 1 / 2 ) * PI;
    m_trajectorySetpoint_pub->publish( msg );
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void ReturnToOrigin::requestVehicleCommand( uint16_t command, float param1, float param2 )
{
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

    VehicleCommand msg{};
    msg.param1           = param1;
    msg.param2           = param2;
    msg.command          = command;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
    request->request     = msg;

    m_serviceDone = false;
    auto result =
        m_vehicleCommand_client->async_send_request( request, std::bind( &ReturnToOrigin::responseCallback, this, std::placeholders::_1 ) );
}

void ReturnToOrigin::onVehicleLocalPosition( const VehicleLocalPosition::SharedPtr msg )
{
    m_localPosition = msg;
}

void ReturnToOrigin::responseCallback( rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future )
{
    auto status = future.wait_for( 1s );

    if( status == std::future_status::ready )
    {
        auto reply      = future.get()->reply;
        m_serviceResult = reply.result;

        switch( m_serviceResult )
        {
            case VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:
            {
                // RCLCPP_INFO( this->get_logger(), "VehicleCommand accepted" );
                break;
            }
            case VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
            {
                RCLCPP_WARN( this->get_logger(), "VehicleCommand temporarily rejected" );
                break;
            }
            case VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:
            {
                RCLCPP_WARN( this->get_logger(), "VehicleCommand denied" );
                break;
            }
            case VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED:
            {
                RCLCPP_WARN( this->get_logger(), "VehicleCommand unsupported" );
                break;
            }
            case VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED:
            {
                RCLCPP_WARN( this->get_logger(), "VehicleCommand failed" );
                break;
            }
            case VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS:
            {
                RCLCPP_WARN( this->get_logger(), "VehicleCommand in progress" );
                break;
            }
            case VehicleCommandAck::VEHICLE_CMD_RESULT_CANCELLED:
            {
                RCLCPP_WARN( this->get_logger(), "VehicleCommand cancelled" );
                break;
            }
            default:
            {
                RCLCPP_WARN( this->get_logger(), "VehicleCommand reply unknown" );
                break;
            }
        }

        m_serviceDone = true;
    }
}

int main( int argc, char* argv[] )
{
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<ReturnToOrigin>() );

    rclcpp::shutdown();
    return 0;
}