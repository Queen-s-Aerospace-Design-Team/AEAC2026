
// #include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Subscriber : public rclcpp::Node
{
  public:
    Subscriber() : Node( "subscriber" )
    {
        auto topic_callback = [this]( std_msgs::msg::String::UniquePtr message )
        { RCLCPP_INFO( get_logger(), "Received Message: %s", message->data.c_str() ); };

        m_subscription = create_subscription<std_msgs::msg::String>( "pubsub_01_msg", 10, topic_callback );
    }

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;
};

int main( int argc, char** argv )
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<Subscriber>() );
    rclcpp::shutdown();
    return 0;
}