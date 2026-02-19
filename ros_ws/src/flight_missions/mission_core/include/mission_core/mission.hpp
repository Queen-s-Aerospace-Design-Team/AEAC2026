#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

class Mission : public rclcpp::Node
{
  public:
    enum FSM : uint8_t
    {
        Init,
        OffboardRequested,
        WaitForStableOffboard,
        ArmRequested,
        MissionObjective,
        FinishPolicyRequested,
        FinishPolicyMonitor,
        Finished,
        Failed
    };

    enum FinishPolicy : uint8_t
    {
        Manual,
        RTL,
        RTLAndDisarm
    };

    explicit Mission( const std::string& nodeName, FinishPolicy finishPolicy = Manual );
    ~Mission() override;

  protected:
    virtual void publishMissionSetpoint()    = 0;
    virtual bool isMissionObjectiveReached() = 0;
    virtual void onMissionObjectiveStart();
    virtual void onMissionFinished();
    virtual void publishOffboardControlMode();

    void requestManualControlMode();
    void requestOffboardControlMode();
    void requestReturnToLaunch();
    void arm();
    void disarm();
    void requestVehicleCommand( uint16_t command, float param1 = 0.0f, float param2 = 0.0f );
    void responseCallback( rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future );
    void onVehicleStatus( const px4_msgs::msg::VehicleStatus::SharedPtr msg );
    void onVehicleLandDetected( const px4_msgs::msg::VehicleLandDetected::SharedPtr msg );
    void runStateMachineTick();
    bool isVehicleArmed() const;
    bool isVehicleLanded() const;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr m_offboardControlMode_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr m_trajectorySetpoint_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr m_vehicleCommand_pub;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr m_vehicleStatus_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr m_vehicleLandDetected_sub;

    // Clients
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr m_vehicleCommand_client;

    px4_msgs::msg::VehicleStatus::SharedPtr m_vehicleStatus;
    px4_msgs::msg::VehicleLandDetected::SharedPtr m_vehicleLandDetected;

    FSM m_state;
    FinishPolicy m_finishPolicy;
    bool m_serviceDone;
    uint8_t m_serviceResult;
    bool m_finishCommandIssued;
    bool m_disarmRequestedAfterLanding;

    rclcpp::TimerBase::SharedPtr m_timer;
};
