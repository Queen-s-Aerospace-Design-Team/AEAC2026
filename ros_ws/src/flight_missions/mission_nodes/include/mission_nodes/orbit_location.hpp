#pragma once

#include "mission_core/mission.hpp"

#include <px4_msgs/msg/vehicle_local_position.hpp>

class OrbitLocation : public Mission
{
  public:
    OrbitLocation();
    ~OrbitLocation() override = default;

  protected:
    void onMissionObjectiveStart() override;
    void publishMissionSetpoint() override;
    bool isMissionObjectiveReached() override;

  private:
    void onVehicleLocalPosition( const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg );

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr m_localPosition_sub;
    px4_msgs::msg::VehicleLocalPosition::SharedPtr m_localPosition;

    bool m_inReachedBand;
    rclcpp::Time m_reachedSince;
    bool m_orbitStarted;
    rclcpp::Time m_orbitStartTime;

    float m_currentSetpointX;
    float m_currentSetpointY;
    float m_currentSetpointZ;
};
