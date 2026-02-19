#pragma once

#include <rclcpp/rclcpp.hpp>

/// @brief Utilities namespace storing useful and generic functions
namespace Utilities
{
    /// @brief Waits for all given clients to verify their services.
    /// @return true or false depending on if clients could connect
    bool waitForServices( rclcpp::Node* node, const std::vector<rclcpp::ClientBase::SharedPtr>& services,
                          std::chrono::seconds timeout = std::chrono::seconds( 1 ) );
} // namespace Utilities