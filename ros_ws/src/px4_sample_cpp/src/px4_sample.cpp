#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class PX4Runner : public rclcpp::Node
{

  public:
};

int main( int argc, char** argv )
{
    (void)argc;
    (void)argv;

    printf( "hello world px4_sample_cpp package\n" );
    return 0;
}
