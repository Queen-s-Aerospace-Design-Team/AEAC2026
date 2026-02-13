#include "mission_core/utilities/utilities.hpp"

namespace Utilities
{
    bool waitForServices( rclcpp::Node* node, const std::vector<rclcpp::ClientBase::SharedPtr>& services,
                                 std::chrono::seconds timeout )
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