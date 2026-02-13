#include "mission_core/mission.hpp"
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace
{
    constexpr float PI                   = 3.14159f;
    constexpr float POSITION_THRESHOLD   = 0.25f;                     // meters
    constexpr std::chrono::seconds DWELL = std::chrono::seconds( 1 ); // must stay within band for this long

    // Origin target
    constexpr float TARGET_X = 0.0f;
    constexpr float TARGET_Y = 0.0f;
    constexpr float TARGET_Z = -2.5f;
} // namespace

class ReturnToOriginV2 : public Mission
{
  public:
    ReturnToOriginV2() : Mission( "return_to_origin_v2", { Mission::RTL, true } ), m_inReachedBand( false )
    {
        rclcpp::QoS bestEffortQoS = rclcpp::QoS( 10 ).best_effort();
        m_localPosition_sub       = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", bestEffortQoS,
            std::bind( &ReturnToOriginV2::onVehicleLocalPosition, this, std::placeholders::_1 ) );
    }

  protected:
    void onMissionObjectiveStart() override { m_inReachedBand = false; }

    void publishMissionSetpoint() override
    {
        TrajectorySetpoint msg{};
        msg.position  = { TARGET_X, TARGET_Y, TARGET_Z };
        msg.yaw       = -1 * PI; // [-PI:PI]
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        m_trajectorySetpoint_pub->publish( msg );
    }

    bool isMissionObjectiveReached() override
    {
        if( !m_localPosition )
        {
            RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 2000, "Waiting for local position..." );
            return false;
        }

        const float dx    = m_localPosition->x - TARGET_X;
        const float dy    = m_localPosition->y - TARGET_Y;
        const float dz    = m_localPosition->z - TARGET_Z;
        const float dist  = std::sqrt( dx * dx + dy * dy + dz * dz );
        const bool inBand = dist < POSITION_THRESHOLD;

        RCLCPP_INFO_THROTTLE( get_logger(), *get_clock(), 1000, "UAV is %f meters to target.", dist );

        if( inBand )
        {
            if( !m_inReachedBand )
            {
                m_inReachedBand = true;
                m_reachedSince  = now();
            }
            else if( ( now() - m_reachedSince ) > rclcpp::Duration( DWELL ) )
            {
                RCLCPP_INFO( get_logger(), "Reached setpoint -> mission accomplished!" );
                return true;
            }
        }
        else
        {
            m_inReachedBand = false;
        }

        return false;
    }

  private:
    void onVehicleLocalPosition( const VehicleLocalPosition::SharedPtr msg ) { m_localPosition = msg; }

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr m_localPosition_sub;
    VehicleLocalPosition::SharedPtr m_localPosition;

    bool m_inReachedBand;
    rclcpp::Time m_reachedSince;
};

int main( int argc, char* argv[] )
{
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<ReturnToOriginV2>() );

    rclcpp::shutdown();
    return 0;
}
