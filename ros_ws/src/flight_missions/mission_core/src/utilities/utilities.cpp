#include "mission_core/utilities/utilities.hpp"

namespace Utilities
{
    bool waitForServices( rclcpp::Node* node, const std::vector<rclcpp::ClientBase::SharedPtr>& services, std::chrono::seconds timeout )
    {

        constexpr uint8_t maxValidationRequests = 120; // 2 min to validate all services
        uint8_t cntr                            = 0;

        for( const auto& service : services )
        {
            RCLCPP_INFO( node->get_logger(), "Waiting for service: '%s'", service->get_service_name() );

            while( !service->wait_for_service( timeout ) )
            {
                if( ++cntr > maxValidationRequests )
                {
                    RCLCPP_ERROR( node->get_logger(), "Validation timed out on service: '%s'", service->get_service_name() );
                    return false;
                }

                if( !rclcpp::ok() )
                {
                    RCLCPP_ERROR( node->get_logger(), "Interrupted while waiting for service: '%s'", service->get_service_name() );
                    return false;
                }

                RCLCPP_WARN( node->get_logger(), "(%ds) Service '%s' not available, waiting again...", cntr, service->get_service_name() );
            }
        }

        return true;
    }
} // namespace Utilities