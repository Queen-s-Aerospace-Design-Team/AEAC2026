import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)
from nav_msgs.msg import Odometry


class cmd_vel_to_px4(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_px4')

        self.declare_parameter('fixed_altitude', -2.0) # Fix altitude of 2.0 meters
        self.declare_parameter('setpoint_rate', 20.0)

        self.fixed_altitude = self.get_parameter('fixed_altitude').value
        self.rate = self.get_parameter('setpoint_rate').value

        self.cmd_vel = Twist()
        self.have_odom = False

        # Subscribers
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info('cmd_vel → PX4 bridge started')

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def odom_callback(self, msg):
        self.have_odom = True

    def timer_callback(self):
        # if not self.have_odom:
        #     return

        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # 1. Offboard control mode
        offboard = OffboardControlMode()
        offboard.timestamp = timestamp
        offboard.position = False
        offboard.velocity = True
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        self.offboard_pub.publish(offboard)

        # 2. Trajectory setpoint
        sp = TrajectorySetpoint()
        sp.timestamp = timestamp

        sp.velocity = [
            float(self.cmd_vel.linear.x),
            float(self.cmd_vel.linear.y),
            0.0 # zero velocity in z-axis (obviosly we want fixed height for this example)
        ]

        sp.position = [
            float('nan'),
            float('nan'),
            self.fixed_altitude
        ]

        sp.yaw = float('nan')
        sp.yawspeed = float(self.cmd_vel.angular.z)

        self.setpoint_pub.publish(sp)

        # 3. Ensure offboard + armed
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1,
            6
        )

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = cmd_vel_to_px4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
