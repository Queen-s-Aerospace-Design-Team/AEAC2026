import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition
)

class cmd_vel_to_px4(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_px4')

        self.declare_parameter('fixed_altitude', -2.0) # Fix altitude of 2.0 meters
        self.declare_parameter('setpoint_rate', 20.0)

        self.fixed_altitude = self.get_parameter('fixed_altitude').value
        self.rate = self.get_parameter('setpoint_rate').value

        self.cmd_vel = Twist()
        self.have_odom = False
        self.vehicle_yaw = None

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
        
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/in/vehicle_local_position',
            self.local_position_callback,
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

        self.get_logger().info('cmd_vel/PX4 bridge started')

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg

    def odom_callback(self, msg: Odometry):
        self.have_odom = msg.pose != None
        
    def local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicle_yaw = msg.heading
    
    def timer_callback(self):
        if self.vehicle_yaw is None:
            self.get_logger().warn("No vehicle heading yet; can't rotate cmd_vel.")
            return

        yaw = self.vehicle_yaw
        
        # Accept FLU from Twist msg
        vx_flu = float(self.cmd_vel.linear.x)
        vy_flu = float(self.cmd_vel.linear.y)
        vz_flu = float(self.cmd_vel.linear.z)

        # FLU -> body-FRD (flip y and z)
        vx_body = vx_flu
        vy_body = -vy_flu
        vz_body = -vz_flu

        # transform to N/E
        v_n = math.cos(yaw) * vx_body - math.sin(yaw) * vy_body
        v_e = math.sin(yaw) * vx_body + math.cos(yaw) * vy_body
        v_d = vz_body

        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        sp.velocity = [v_n, v_e, 0.0 * v_d]  # keep z velocity 0 if you're holding altitude via position
        sp.position = [None, None, self.fixed_altitude]

        sp.yaw = None
        sp.yawspeed = float(self.cmd_vel.angular.z)

        self.setpoint_pub.publish(sp)

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1,
            6
        )

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

    def send_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
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
