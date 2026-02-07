#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <stdint.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**
 * @brief PX4 Example class to test and experiment with px4 functionlaity. Copied from
 * px4_ros_com/src/examples/offboard/offboard_control.cpp.
 */

namespace
{
    constexpr float PI                   = 3.14159f;
    constexpr float POSITION_THRESHOLD   = 0.25f; // meters
    constexpr float VELOCITY_THRESHOLD   = 0.30f; // m/s
    constexpr std::chrono::seconds DWELL = 1s;    // must stay within band for this long
    constexpr float TARGET_X             = 0.0f;
    constexpr float TARGET_Y             = 0.0f;
    constexpr float TARGET_Z             = -2.5f;

    enum State : uint8_t
    {
        Warmup,
        Offboard,
        Done
    };
} // namespace

class PX4Example : public rclcpp::Node
{
  public:
    PX4Example() : Node( "offboard_control" )
    {
        rclcpp::QoS bestEffortQoS = rclcpp::QoS( 10 ).best_effort();

        m_offboardControlMode_pub = this->create_publisher<OffboardControlMode>( "/fmu/in/offboard_control_mode", 10 );
        m_trajectorySetpoint_pub  = this->create_publisher<TrajectorySetpoint>( "/fmu/in/trajectory_setpoint", 10 );
        m_vehicleCommand_pub      = this->create_publisher<VehicleCommand>( "/fmu/in/vehicle_command", 10 );

        m_localPosition_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", bestEffortQoS, std::bind( &PX4Example::onVehicleLocalPosition, this, std::placeholders::_1 ) );

        m_offboardCounter = 0;

        auto state_timer_callback = [this]() -> void
        {
            switch( m_state )
            {
                case Done:
                {
                    m_timer->cancel(); // stop this timer callback

                    RCLCPP_INFO( this->get_logger(), "Reached setpoint -> exiting Offboard (POSCTL)." );
                    break;
                }
                case Warmup:
                {
                    // offboard_control_mode needs to be paired with trajectory_setpoint
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint();

                    if( m_offboardCounter == 10 )
                    {
                        this->publish_vehicle_command( VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6 ); // Offboard
                        this->arm();
                        m_state = State::Offboard;
                    }

                    if( m_offboardCounter < 11 )
                    {
                        m_offboardCounter++;
                    }

                    break;
                }
                case Offboard:
                {
                    if( !m_localPosition )
                    {
                        break;
                    }

                    publish_offboard_control_mode();
                    publish_trajectory_setpoint();

                    const float dx    = m_localPosition->x - TARGET_X;
                    const float dy    = m_localPosition->y - TARGET_Y;
                    const float dz    = m_localPosition->z - TARGET_Z;
                    const float dist  = std::sqrt( dx * dx + dy * dy + dz * dz );
                    const bool inBand = dist < POSITION_THRESHOLD;

                    RCLCPP_INFO( this->get_logger(), "UAV is %f meters to target.", dist );

                    if( inBand )
                    {
                        if( !m_inReachedBand )
                        {
                            m_inReachedBand = true;
                            m_reachedSince  = now();
                        }
                        else if( ( now() - m_reachedSince ) > rclcpp::Duration( DWELL ) )
                        {
                            // Exit Offboard
                            this->publish_vehicle_command( VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3 );
                            m_state = State::Done;
                        }
                    }
                    else
                    {
                        m_inReachedBand = false;
                    }
                }
            }
        };

        m_timer = create_wall_timer( 100ms, state_timer_callback );
    }

    ~PX4Example() override { RCLCPP_INFO( this->get_logger(), "%s destructor called.", this->get_name() ); }

    void arm();
    void disarm();

  private:
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command( uint16_t command, float param1 = 0.0, float param2 = 0.0 );
    void onVehicleLocalPosition( const VehicleLocalPosition::SharedPtr msg );

    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr m_offboardControlMode_pub;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr m_trajectorySetpoint_pub;
    rclcpp::Publisher<VehicleCommand>::SharedPtr m_vehicleCommand_pub;

    // Subscriptions
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr m_localPosition_sub;

    VehicleLocalPosition::SharedPtr m_localPosition;
    bool m_inReachedBand = false;
    rclcpp::Time m_reachedSince;

    rclcpp::TimerBase::SharedPtr m_timer;

    uint8_t m_offboardCounter; //!< counter for the number of setpoints sent

    State m_state = State::Warmup;
};

void PX4Example::arm()
{
    publish_vehicle_command( VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0 );

    RCLCPP_INFO( this->get_logger(), "Arm command send" );
}

void PX4Example::disarm()
{
    publish_vehicle_command( VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0 );

    RCLCPP_INFO( this->get_logger(), "Disarm command send" );
}

void PX4Example::publish_offboard_control_mode()
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

void PX4Example::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    msg.position = { 0.0, 0.0, -2.5 }; // Position 0, 0, and 2.5m in the world
    // msg.velocity  = { 0 };              // { 5.0f, 5.0f, -1.0f };
    msg.yaw       = -1 * PI; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // msg.yawspeed  = ( 1 / 2 ) * PI;
    m_trajectorySetpoint_pub->publish( msg );
}

void PX4Example::publish_vehicle_command( uint16_t command, float param1, float param2 )
{
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
    m_vehicleCommand_pub->publish( msg );
}

void PX4Example::onVehicleLocalPosition( const VehicleLocalPosition::SharedPtr msg )
{
    m_localPosition = msg;
}

int main( int argc, char* argv[] )
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<PX4Example>() );

    rclcpp::shutdown();
    return 0;
}