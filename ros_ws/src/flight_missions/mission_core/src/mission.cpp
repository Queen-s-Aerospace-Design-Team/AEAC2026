#include "mission_core/mission.hpp"

#include "mission_core/utilities/utilities.hpp"

#include <chrono>
#include <future>
#include <vector>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

Mission::Mission( const std::string& nodeName, FinishPolicy finishPolicy )
    : Node( nodeName )
    , m_offboardControlMode_pub( create_publisher<OffboardControlMode>( "/fmu/in/offboard_control_mode", 10 ) )
    , m_trajectorySetpoint_pub( create_publisher<TrajectorySetpoint>( "/fmu/in/trajectory_setpoint", 10 ) )
    , m_vehicleCommand_pub( create_publisher<VehicleCommand>( "/fmu/in/vehicle_command", 10 ) )
    , m_vehicleCommand_client( create_client<px4_msgs::srv::VehicleCommand>( "/fmu/vehicle_command" ) )
    , m_state( FSM::Init )
    , m_finishPolicy( finishPolicy )
    , m_serviceDone( false )
    , m_serviceResult( 0 )
    , m_finishCommandIssued( false )
    , m_disarmRequestedAfterLanding( false )
{
    RCLCPP_INFO( get_logger(), "Starting mission: %s", get_name() );

    if( !Utilities::waitForServices( this, { m_vehicleCommand_client } ) )
    {
        RCLCPP_ERROR( get_logger(), "Failed to validate services. Exiting..." );
        m_state = FSM::Failed;
    }

    m_vehicleStatus_sub       = create_subscription<VehicleStatus>( "/fmu/out/vehicle_status", 10,
                                                              std::bind( &Mission::onVehicleStatus, this, std::placeholders::_1 ) );
    m_vehicleLandDetected_sub = create_subscription<VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", 10, std::bind( &Mission::onVehicleLandDetected, this, std::placeholders::_1 ) );

    m_timer = create_wall_timer( 100ms, std::bind( &Mission::runStateMachineTick, this ) );
}

Mission::~Mission()
{
    RCLCPP_INFO( get_logger(), "%s destructor called.", get_name() );
}

void Mission::onMissionObjectiveStart()
{
}

void Mission::onMissionFinished()
{
}

void Mission::requestOffboardControlMode()
{
    RCLCPP_INFO( get_logger(), "Requesting switch to Offboard mode..." );
    requestVehicleCommand( VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6 );
}

void Mission::requestManualControlMode()
{
    RCLCPP_INFO( get_logger(), "Requesting switch to Manual mode..." );
    requestVehicleCommand( VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 3 );
}

void Mission::requestReturnToLaunch()
{
    RCLCPP_INFO( get_logger(), "Requesting Return to Launch (RTL)..." );
    requestVehicleCommand( VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH );
}

void Mission::arm()
{
    RCLCPP_INFO( get_logger(), "Arm command send..." );
    requestVehicleCommand( VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f );
}

void Mission::disarm()
{
    RCLCPP_INFO( get_logger(), "Disarm command send..." );
    requestVehicleCommand( VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f );
}

void Mission::publishOffboardControlMode()
{
    OffboardControlMode msg{};
    msg.position     = true;
    msg.velocity     = false;
    msg.acceleration = false;
    msg.attitude     = false;
    msg.body_rate    = false;
    msg.timestamp    = get_clock()->now().nanoseconds() / 1000;
    m_offboardControlMode_pub->publish( msg );
}

void Mission::requestVehicleCommand( uint16_t command, float param1, float param2 )
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
    msg.timestamp        = get_clock()->now().nanoseconds() / 1000;
    request->request     = msg;

    m_serviceDone = false;
    m_vehicleCommand_client->async_send_request( request, std::bind( &Mission::responseCallback, this, std::placeholders::_1 ) );
}

void Mission::responseCallback( rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future )
{
    auto status = future.wait_for( 1s );

    if( status == std::future_status::ready )
    {
        auto reply      = future.get()->reply;
        m_serviceResult = reply.result;

        switch( m_serviceResult )
        {
            case VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:
                break;
            case VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                RCLCPP_WARN( get_logger(), "VehicleCommand temporarily rejected" );
                break;
            case VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:
                RCLCPP_WARN( get_logger(), "VehicleCommand denied" );
                break;
            case VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED:
                RCLCPP_WARN( get_logger(), "VehicleCommand unsupported" );
                break;
            case VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED:
                RCLCPP_WARN( get_logger(), "VehicleCommand failed" );
                break;
            case VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS:
                RCLCPP_WARN( get_logger(), "VehicleCommand in progress" );
                break;
            case VehicleCommandAck::VEHICLE_CMD_RESULT_CANCELLED:
                RCLCPP_WARN( get_logger(), "VehicleCommand cancelled" );
                break;
            default:
                RCLCPP_WARN( get_logger(), "VehicleCommand reply unknown" );
                break;
        }

        m_serviceDone = true;
    }
}

void Mission::onVehicleStatus( const VehicleStatus::SharedPtr msg )
{
    m_vehicleStatus = msg;
}

void Mission::onVehicleLandDetected( const VehicleLandDetected::SharedPtr msg )
{
    m_vehicleLandDetected = msg;
}

bool Mission::isVehicleArmed() const
{
    return static_cast<bool>( m_vehicleStatus ) && ( m_vehicleStatus->arming_state == VehicleStatus::ARMING_STATE_ARMED );
}

bool Mission::isVehicleLanded() const
{
    return static_cast<bool>( m_vehicleLandDetected ) && m_vehicleLandDetected->landed;
}

void Mission::runStateMachineTick()
{
    if( m_state < FSM::FinishPolicyRequested )
    {
        publishOffboardControlMode();
        publishMissionSetpoint();
    }

    switch( m_state )
    {
        case FSM::Init:
        {
            if( !m_vehicleCommand_client || !m_vehicleCommand_client->service_is_ready() )
            {
                m_state = FSM::Failed;
                break;
            }

            requestOffboardControlMode();
            m_state = FSM::OffboardRequested;
            break;
        }
        case FSM::OffboardRequested:
        {
            if( m_serviceDone )
            {
                if( m_serviceResult == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED )
                {
                    RCLCPP_INFO( get_logger(), "Entered offboard mode" );
                    m_state = FSM::WaitForStableOffboard;
                }
                else
                {
                    RCLCPP_ERROR( get_logger(), "Failed to enter offboard mode, exiting..." );
                    m_state = FSM::Failed;
                }
            }

            break;
        }
        case FSM::WaitForStableOffboard:
        {
            static uint8_t offboardCounter = 0;

            if( ++offboardCounter > 10 )
            {
                arm();
                m_state = FSM::ArmRequested;
            }

            break;
        }
        case FSM::ArmRequested:
        {
            if( m_serviceDone )
            {
                if( m_serviceResult == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED )
                {
                    RCLCPP_INFO( get_logger(), "UAV armed" );
                    onMissionObjectiveStart();
                    m_state = FSM::MissionObjective;
                }
                else
                {
                    RCLCPP_ERROR( get_logger(), "Failed to arm. Exiting..." );
                    m_state = FSM::Failed;
                }
            }

            break;
        }
        case FSM::MissionObjective:
        {
            if( isMissionObjectiveReached() )
            {
                RCLCPP_INFO( get_logger(), "Mission objective reached -> transitioning to finish policy..." );
                m_finishCommandIssued         = false;
                m_disarmRequestedAfterLanding = false;
                m_state                       = FSM::FinishPolicyRequested;
            }

            break;
        }
        case FSM::FinishPolicyRequested:
        {
            if( !m_finishCommandIssued )
            {
                switch( m_finishPolicy )
                {
                    case FinishPolicy::Manual:
                        requestManualControlMode();
                        break;
                    case FinishPolicy::RTL:
                    case FinishPolicy::RTLAndDisarm:
                        requestReturnToLaunch();
                        break;
                }

                m_finishCommandIssued = true;
                break;
            }

            // Wait for service to complete
            if( m_serviceDone )
            {
                if( m_serviceResult == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED )
                {
                    switch( m_finishPolicy )
                    {
                        case FinishPolicy::Manual:
                            RCLCPP_INFO( get_logger(), "Manual mode resumed" );
                            break;
                        case FinishPolicy::RTL:
                        case FinishPolicy::RTLAndDisarm:
                            RCLCPP_INFO( get_logger(), "RTL accepted. Monitoring landing/disarm status..." );
                            break;
                    }

                    m_state = FSM::FinishPolicyMonitor;
                }
                else
                {
                    RCLCPP_ERROR( get_logger(), "Failed to execute finish policy command. Failing mission..." );
                    m_state = FSM::Failed;
                }
            }

            break;
        }
        case FSM::FinishPolicyMonitor:
        {
            const bool landed = isVehicleLanded();
            const bool armed  = isVehicleArmed();

            switch( m_finishPolicy )
            {
                case FinishPolicy::Manual:
                    m_state = FSM::Finished;
                    break;
                case FinishPolicy::RTL:
                    if( landed && !armed )
                    {
                        m_state = FSM::Finished;
                    }
                    break;
                case FinishPolicy::RTLAndDisarm:
                    if( landed && armed && !m_disarmRequestedAfterLanding )
                    {
                        disarm();
                        m_disarmRequestedAfterLanding = true;
                        break;
                    }

                    if( landed && !armed )
                    {
                        m_state = FSM::Finished;
                    }
                    break;
            }

            break;
        }
        case FSM::Finished:
        {
            m_timer->cancel();
            onMissionFinished();
            RCLCPP_INFO( get_logger(), "Mission Completed! Mission timer callback stopped." );
            break;
        }
        case FSM::Failed:
        {
            m_timer->cancel();
            RCLCPP_ERROR( get_logger(), "Mission Failed! Mission timer callback stopped. Exiting..." );
            rclcpp::shutdown();
            break;
        }
    }
}
