#pragma once

#include <rclcpp/rclcpp.hpp>

class OperationManager : public rclcpp::Node
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

    explicit OperationManager( const std::string& nodeName, FinishPolicy finishPolicy = Manual );
    ~OperationManager() override = default;

    
};