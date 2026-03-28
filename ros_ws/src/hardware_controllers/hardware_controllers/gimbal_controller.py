import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger
from typing import Optional, Tuple

try:
    import Jetson.GPIO as GPIO
    JETSON_GPIO_AVAILABLE = True
except ImportError:
    GPIO = None
    JETSON_GPIO_AVAILABLE = False

class GimbalController(Node):
    """
    Temporary target topic contract using geometry_msgs/Vector3:
      x = signed vertical angle error to target [deg]
      y = signed relative vertical position [m]
      z = target depth [m]

    Replace Vector3 with your final perception message once it exists.
    """

    def __init__(self) -> None:
        super().__init__("gimbal_controller")

        # General / hardware
        self.declare_parameter("dry_run", False)
        self.declare_parameter("pitch_pwm_pin", 33)
        self.declare_parameter("water_gpio_pin", 31)
        self.declare_parameter("gpio_numbering_mode", "BOARD")
        self.declare_parameter("water_active_high", True)

        # Pitch servo / one-axis gimbal
        self.declare_parameter("servo_pwm_frequency_hz", 50.0)
        self.declare_parameter("servo_min_pitch_deg", -30.0)
        self.declare_parameter("servo_max_pitch_deg", 30.0)
        self.declare_parameter("servo_min_pulse_us", 500.0)
        self.declare_parameter("servo_max_pulse_us", 2500.0)
        self.declare_parameter("initial_pitch_deg", 0.0)
        self.declare_parameter("servo_gain", 0.6)
        self.declare_parameter("max_servo_step_deg", 5.0)

        # Lock-on / firing logic
        self.declare_parameter("camera_to_nozzle_offset_deg", 0.0)
        self.declare_parameter("invert_target_angle", False)
        self.declare_parameter("target_timeout_s", 0.5)
        self.declare_parameter("lock_angle_threshold_deg", 2.0)
        self.declare_parameter("lock_position_threshold_m", 0.10)
        self.declare_parameter("min_shoot_depth_m", 1.0)
        self.declare_parameter("max_shoot_depth_m", 4.0)
        self.declare_parameter("fire_duration_s", 0.75)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.pitch_pwm_pin = int(self.get_parameter("pitch_pwm_pin").value)
        self.water_gpio_pin = int(self.get_parameter("water_gpio_pin").value)
        self.gpio_numbering_mode = str(self.get_parameter("gpio_numbering_mode").value).upper()
        self.water_active_high = bool(self.get_parameter("water_active_high").value)

        self.servo_pwm_frequency_hz = float(self.get_parameter("servo_pwm_frequency_hz").value)
        self.servo_min_pitch_deg = float(self.get_parameter("servo_min_pitch_deg").value)
        self.servo_max_pitch_deg = float(self.get_parameter("servo_max_pitch_deg").value)
        self.servo_min_pulse_us = float(self.get_parameter("servo_min_pulse_us").value)
        self.servo_max_pulse_us = float(self.get_parameter("servo_max_pulse_us").value)
        self.servo_gain = float(self.get_parameter("servo_gain").value)
        self.max_servo_step_deg = float(self.get_parameter("max_servo_step_deg").value)

        self.camera_to_nozzle_offset_deg = float(self.get_parameter("camera_to_nozzle_offset_deg").value)
        self.invert_target_angle = bool(self.get_parameter("invert_target_angle").value)
        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.lock_angle_threshold_deg = float(self.get_parameter("lock_angle_threshold_deg").value)
        self.lock_position_threshold_m = float(self.get_parameter("lock_position_threshold_m").value)
        self.min_shoot_depth_m = float(self.get_parameter("min_shoot_depth_m").value)
        self.max_shoot_depth_m = float(self.get_parameter("max_shoot_depth_m").value)
        self.fire_duration_s = float(self.get_parameter("fire_duration_s").value)

        self.current_pitch_deg = self._clamp(
            float(self.get_parameter("initial_pitch_deg").value),
            self.servo_min_pitch_deg,
            self.servo_max_pitch_deg,
        )

        self.last_target_time_s: Optional[float] = None
        self.last_angle_error_deg: Optional[float] = None
        self.last_relative_position_m: Optional[float] = None
        self.last_depth_m: Optional[float] = None

        self.gpio_ready = False
        self.pitch_pwm = None

        self._setup_gpio()
        self._command_pitch(self.current_pitch_deg)

        # Temporary perception input until your real target message exists
        self.target_sub = self.create_subscription(
            Vector3,
            "/perception/target_info",
            self.target_callback,
            10,
        )

        # Fire command service
        self.shoot_srv = self.create_service(
            Trigger,
            "/shoot_water",
            self.shoot_callback,
        )

        self.get_logger().info("gimbal_controller ready")

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _setup_gpio(self) -> None:
        if self.dry_run:
            self.get_logger().warn("Running in dry_run mode; no GPIO will be driven.")
            return

        if not JETSON_GPIO_AVAILABLE:
            self.get_logger().error(
                "Jetson.GPIO is not available. Install python3-jetson-gpio "
                "or run with dry_run:=true."
            )
            return

        GPIO.setwarnings(False)

        if self.gpio_numbering_mode == "BCM":
            GPIO.setmode(GPIO.BCM)
        else:
            GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.water_gpio_pin, GPIO.OUT)
        GPIO.setup(self.pitch_pwm_pin, GPIO.OUT)

        self.pitch_pwm = GPIO.PWM(self.pitch_pwm_pin, self.servo_pwm_frequency_hz)
        self.pitch_pwm.start(self._angle_to_duty_cycle(self.current_pitch_deg))

        self.gpio_ready = True
        self._set_water(False)

    def _angle_to_duty_cycle(self, pitch_deg: float) -> float:
        pitch_deg = self._clamp(
            pitch_deg,
            self.servo_min_pitch_deg,
            self.servo_max_pitch_deg,
        )

        span_deg = self.servo_max_pitch_deg - self.servo_min_pitch_deg
        if span_deg <= 0.0:
            raise ValueError("servo_max_pitch_deg must be greater than servo_min_pitch_deg")

        normalized = (pitch_deg - self.servo_min_pitch_deg) / span_deg
        pulse_us = self.servo_min_pulse_us + normalized * (
            self.servo_max_pulse_us - self.servo_min_pulse_us
        )

        period_us = 1_000_000.0 / self.servo_pwm_frequency_hz
        return 100.0 * pulse_us / period_us

    def _command_pitch(self, pitch_deg: float) -> None:
        pitch_deg = self._clamp(
            pitch_deg,
            self.servo_min_pitch_deg,
            self.servo_max_pitch_deg,
        )
        self.current_pitch_deg = pitch_deg

        if self.dry_run or not self.gpio_ready or self.pitch_pwm is None:
            self.get_logger().debug(f"Pitch command: {pitch_deg:.2f} deg")
            return

        duty_cycle = self._angle_to_duty_cycle(pitch_deg)
        self.pitch_pwm.ChangeDutyCycle(duty_cycle)

    def _set_water(self, enabled: bool) -> None:
        gpio_value = GPIO.HIGH if (enabled == self.water_active_high) else GPIO.LOW

        if self.dry_run or not self.gpio_ready:
            self.get_logger().debug(f"Water output set to {enabled}")
            return

        GPIO.output(self.water_gpio_pin, gpio_value)

    def _has_fresh_target(self) -> bool:
        if self.last_target_time_s is None:
            return False
        return (self._now_s() - self.last_target_time_s) <= self.target_timeout_s

    def _shoot_ready_status(self) -> Tuple[bool, str]:
        if not self._has_fresh_target():
            return False, "No recent target message."

        if (
            self.last_angle_error_deg is None
            or self.last_relative_position_m is None
            or self.last_depth_m is None
        ):
            return False, "Target state is incomplete."

        if abs(self.last_angle_error_deg) > self.lock_angle_threshold_deg:
            return (
                False,
                f"Angle error {self.last_angle_error_deg:.2f} deg exceeds "
                f"threshold {self.lock_angle_threshold_deg:.2f} deg.",
            )

        if abs(self.last_relative_position_m) > self.lock_position_threshold_m:
            return (
                False,
                f"Relative position {self.last_relative_position_m:.3f} exceeds "
                f"threshold {self.lock_position_threshold_m:.3f}.",
            )

        if not (self.min_shoot_depth_m <= self.last_depth_m <= self.max_shoot_depth_m):
            return (
                False,
                f"Depth {self.last_depth_m:.2f} m is outside "
                f"[{self.min_shoot_depth_m:.2f}, {self.max_shoot_depth_m:.2f}] m.",
            )

        return True, "Target lock confirmed."

    def target_callback(self, msg: Vector3) -> None:
        # Assume msg as: [angle, relative_pos, depth]
        angle_error_deg = float(msg.x)
        relative_position_m = float(msg.y)
        depth_m = float(msg.z)

        if self.invert_target_angle:
            angle_error_deg *= -1.0

        angle_error_deg += self.camera_to_nozzle_offset_deg

        step_deg = self.servo_gain * angle_error_deg
        step_deg = self._clamp(
            step_deg,
            -self.max_servo_step_deg,
            self.max_servo_step_deg,
        )
        self._command_pitch(self.current_pitch_deg + step_deg)

        self.last_target_time_s = self._now_s()
        self.last_angle_error_deg = angle_error_deg
        self.last_relative_position_m = relative_position_m
        self.last_depth_m = depth_m

    def shoot_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        ready, reason = self._shoot_ready_status()
        if not ready:
            response.success = False
            response.message = reason
            self.get_logger().warn(f"Shoot request rejected: {reason}")
            return response

        self.get_logger().info("Target lock confirmed. Firing water output.")
        try:
            self._set_water(True)
            time.sleep(self.fire_duration_s)
        finally:
            self._set_water(False)

        response.success = True
        response.message = f"Water fired for {self.fire_duration_s:.2f} s."
        return response

    def shutdown_hardware(self) -> None:
        try:
            self._set_water(False)
        except Exception:
            pass

        if self.pitch_pwm is not None:
            try:
                self.pitch_pwm.stop()
            except Exception:
                pass

        if self.gpio_ready and JETSON_GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except Exception:
                pass

        self.gpio_ready = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GimbalController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hardware()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()