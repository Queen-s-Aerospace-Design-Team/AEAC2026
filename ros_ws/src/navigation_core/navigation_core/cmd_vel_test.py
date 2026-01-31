import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

ORBIT = 'orbit'
TRIANGLE = 'triangle'

class cmd_vel_test(Node):
    def __init__(self):
        super().__init__('cmd_vel_test')

        # Parameters
        self.declare_parameter(ORBIT, True)
        self.declare_parameter(TRIANGLE, False)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('segment_time', 5.0)
        self.declare_parameter('twist_rate', 20.0)

        self.orbit = self.get_parameter(ORBIT).value
        self.triangle = self.get_parameter(TRIANGLE).value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.segment_time = self.get_parameter('segment_time').value
        self.rate = self.get_parameter('twist_rate').value

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Internal state
        self.cmd = Twist()
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate,self.timer_callback)

        self.get_logger().info('cmd_vel test pattern generator started')

    # -------------------------

    def timer_callback(self):

        if self.orbit:
            self.orbit_motion()
        elif self.triangle:
            self.triangle_motion()
        else:
            self.cmd = Twist()  # Stop

        self.cmd_pub.publish(self.cmd)

    # -------------------------

    def orbit_motion(self):
        """
        Circular motion:
        v = constant
        yaw_rate = constant
        """
        self.cmd.linear.x = self.linear_speed
        self.cmd.angular.z = self.angular_speed

    # -------------------------

    def triangle_motion(self):

        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        side_time = self.segment_time
        turn_time = (2.094) / self.angular_speed   # 120 deg

        cycle_time = 3 * (side_time + turn_time)
        t_cycle = t % cycle_time

        phase = int(t_cycle / (side_time + turn_time))

        t_phase = t_cycle - phase * (side_time + turn_time)

        if t_phase < side_time:
            # Fly straight
            self.cmd.linear.x = self.linear_speed
            self.cmd.angular.z = 0.0
        else:
            # Turn
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = self.angular_speed

def main(args=None):
    rclpy.init(args=args)
    node = cmd_vel_test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
