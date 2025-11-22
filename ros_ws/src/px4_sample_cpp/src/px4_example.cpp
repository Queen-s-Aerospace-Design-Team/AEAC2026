#include <chrono>
#include <iostream>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#define PI 3.14159f

/**
 * @brief PX4 Example class to test and experiment with px4 functionlaity. Copied from
 * px4_ros_com/src/examples/offboard/offboard_control.cpp.
 */

class PX4Example : public rclcpp::Node
{
  public:
    PX4Example() : Node( "offboard_control" )
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>( "/fmu/in/offboard_control_mode", 10 );
        trajectory_setpoint_publisher_   = this->create_publisher<TrajectorySetpoint>( "/fmu/in/trajectory_setpoint", 10 );
        vehicle_command_publisher_       = this->create_publisher<VehicleCommand>( "/fmu/in/vehicle_command", 10 );

        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void
        {
            if( offboard_setpoint_counter_ == 10 )
            {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command( VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6 );

                // Arm the vehicle
                this->arm();
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();
            publish_trajectory_setpoint();

            // stop the counter after reaching 11
            if( offboard_setpoint_counter_ < 11 )
            {
                offboard_setpoint_counter_++;
            }
        };

        // This function (timer_callback) gets launched every 100ms.
        timer_ = create_wall_timer( 100ms, timer_callback );
    }

    ~PX4Example() override { RCLCPP_INFO( this->get_logger(), "PX4Example destructor called." ); }

    void arm();
    void disarm();

  private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    std::atomic<uint64_t> timestamp_; //!< common synced timestamped

    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command( uint16_t command, float param1 = 0.0, float param2 = 0.0 );
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
    offboard_control_mode_publisher_->publish( msg );
}

void PX4Example::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    msg.position = { 0.0, 0.0, -2.5 }; // Position 0, 0, and 2.5m in the world
    // msg.velocity  = { 0 };              // { 5.0f, 5.0f, -1.0f };
    msg.yaw       = -1 * PI; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // msg.yawspeed  = ( 1 / 2 ) * PI;
    trajectory_setpoint_publisher_->publish( msg );
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
    vehicle_command_publisher_->publish( msg );
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