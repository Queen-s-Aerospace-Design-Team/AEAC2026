#include <chrono>
#include <memory>
#include <string>

// ROS2 includes

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace
{
    constexpr const char *DEFAULT_MESSAGE = "Hello, world! ";
}

class Publisher : public rclcpp::Node
{
public:
    Publisher() : Node("publisher"), m_count(0)
    {
        auto timer_callback = [this]() -> void
        {
            auto message = std_msgs::msg::String();
            message.data = DEFAULT_MESSAGE + std::to_string(m_count++);

            // Log that we are publishing a message.
            RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());

            // Use our publisher's message type to send a message.
            m_publisher->publish(message);
        };

        m_timer = create_wall_timer(std::chrono::milliseconds(250), timer_callback);
        m_publisher = create_publisher<std_msgs::msg::String>("pubsub_01_msg", 10);
    }

private:
    // Some member variables
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    size_t m_count;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
    // Something was added here!!!
}
