#include "mission_nodes/orbit_location.hpp"

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>

using namespace std::chrono_literals;

namespace
{
    constexpr float PI                   = 3.14159f;
    constexpr float POSITION_THRESHOLD   = 0.35f;                     // meters
    constexpr std::chrono::seconds DWELL = std::chrono::seconds( 1 ); // must stay within band for this long

    // Orbit center target
    constexpr float ORBIT_TARGET_X = 0.0f;
    constexpr float ORBIT_TARGET_Y = 0.0f;
    constexpr float ORBIT_TARGET_Z = -2.5f;

    // Orbit shape and timing
    constexpr float ORBIT_RADIUS        = 2.0f;  // meters
    constexpr float ORBIT_ANGULAR_SPEED = 0.35f; // rad/s
    constexpr float ORBITS_TO_COMPLETE  = 1.0f;  // number of orbits to complete
} // namespace

using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleLocalPosition;

OrbitLocation::OrbitLocation()
    : Mission( "orbit_location", Mission::RTLAndDisarm )
    , m_inReachedBand( false )
    , m_orbitStarted( false )
    , m_currentSetpointX( ORBIT_TARGET_X + ORBIT_RADIUS )
    , m_currentSetpointY( ORBIT_TARGET_Y )
    , m_currentSetpointZ( ORBIT_TARGET_Z )
{
    rclcpp::QoS bestEffortQoS = rclcpp::QoS( 10 ).best_effort();
    m_localPosition_sub =
        create_subscription<VehicleLocalPosition>( "/fmu/out/vehicle_local_position", bestEffortQoS,
                                                   std::bind( &OrbitLocation::onVehicleLocalPosition, this, std::placeholders::_1 ) );
}

void OrbitLocation::onMissionObjectiveStart()
{
    m_inReachedBand = false;
    m_orbitStarted  = false;
}

void OrbitLocation::publishMissionSetpoint()
{
    if( !m_orbitStarted )
    {
        m_currentSetpointX = ORBIT_TARGET_X + ORBIT_RADIUS;
        m_currentSetpointY = ORBIT_TARGET_Y;
        m_currentSetpointZ = ORBIT_TARGET_Z;
    }
    else
    {
        const double elapsedSec = ( now() - m_orbitStartTime ).nanoseconds() / 1'000'000'000.0;
        const float angle       = static_cast<float>( ORBIT_ANGULAR_SPEED * elapsedSec );
        m_currentSetpointX      = ORBIT_TARGET_X + ORBIT_RADIUS * std::cos( angle );
        m_currentSetpointY      = ORBIT_TARGET_Y + ORBIT_RADIUS * std::sin( angle );
        m_currentSetpointZ      = ORBIT_TARGET_Z;
    }

    TrajectorySetpoint msg;
    msg.position  = { m_currentSetpointX, m_currentSetpointY, m_currentSetpointZ };
    msg.yaw       = -1 * PI; // [-PI:PI]
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    m_trajectorySetpoint_pub->publish( msg );
}

bool OrbitLocation::isMissionObjectiveReached()
{
    if( !m_localPosition )
    {
        RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 2000, "Waiting for local position..." );
        return false;
    }

    if( !m_orbitStarted )
    {
        const float dxToEntry      = m_localPosition->x - ( ORBIT_TARGET_X + ORBIT_RADIUS );
        const float dyToEntry      = m_localPosition->y - ORBIT_TARGET_Y;
        const float dzToEntry      = m_localPosition->z - ORBIT_TARGET_Z;
        const float distToEntry    = std::sqrt( dxToEntry * dxToEntry + dyToEntry * dyToEntry + dzToEntry * dzToEntry );
        const bool inEntryPosition = distToEntry < POSITION_THRESHOLD;

        RCLCPP_INFO_THROTTLE( get_logger(), *get_clock(), 1000, "UAV is %f meters to orbit entry setpoint.", distToEntry );

        if( inEntryPosition )
        {
            if( !m_inReachedBand )
            {
                m_inReachedBand = true;
                m_reachedSince  = now();
            }
            else if( ( now() - m_reachedSince ) > rclcpp::Duration( DWELL ) )
            {
                m_orbitStarted   = true;
                m_orbitStartTime = now();
                m_inReachedBand  = false;
                RCLCPP_INFO( get_logger(), "Orbit entry reached. Starting orbit objective..." );
            }
        }
        else
        {
            m_inReachedBand = false;
        }

        return false;
    }

    const double orbitElapsedSec = ( now() - m_orbitStartTime ).nanoseconds() / 1'000'000'000.0;
    const double orbitTravelled  = ORBIT_ANGULAR_SPEED * orbitElapsedSec;
    const double orbitRequired   = 2.0 * PI * ORBITS_TO_COMPLETE;

    RCLCPP_INFO_THROTTLE( get_logger(), *get_clock(), 1000, "Orbit progress: %.2f / %.2f rad", orbitTravelled, orbitRequired );

    if( orbitTravelled >= orbitRequired )
    {
        RCLCPP_INFO( get_logger(), "Orbit objective completed -> requesting finish policy..." );
        return true;
    }

    return false;
}

void OrbitLocation::onVehicleLocalPosition( const VehicleLocalPosition::SharedPtr msg )
{
    m_localPosition = msg;
}

int main( int argc, char* argv[] )
{
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<OrbitLocation>() );

    rclcpp::shutdown();
    return 0;
}
