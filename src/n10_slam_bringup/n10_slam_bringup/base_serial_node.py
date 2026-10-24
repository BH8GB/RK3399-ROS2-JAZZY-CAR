import math
from typing import Optional

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
from std_msgs.msg import Float32MultiArray, UInt16
from tf2_ros import TransformBroadcaster

from .base_protocol import (
    CMD_TELEMETRY,
    FrameParser,
    Telemetry,
    decode_telemetry,
    encode_stop,
    encode_wheel_rpm,
    STATUS_IMU_VALID,
    STATUS_STALL_PROTECTED,
    STATUS_UART_RECOVERED,
    STATUS_WATCHDOG_STOPPED,
)
from .mecanum import (
    limit_wheel_rpm,
    twist_to_wheel_rpm,
    wheel_distances_to_body_delta,
)


def _normalise_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _counter_delta(current: int, previous: int) -> int:
    return ((current - previous + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)


def _signs(values: list[int], parameter_name: str) -> tuple[int, int, int, int]:
    if len(values) != 4 or any(value not in (-1, 1) for value in values):
        raise ValueError(f'{parameter_name} must contain four values, each +1 or -1')
    return tuple(values)


class BaseSerialNode(Node):
    def __init__(self) -> None:
        super().__init__('base_serial')

        self.declare_parameter('port', '/dev/robot_base')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_diameter', 0.075)
        self.declare_parameter('half_length', 0.040)
        self.declare_parameter('half_width', 0.040)
        self.declare_parameter('counts_per_revolution', 60000)
        self.declare_parameter('max_wheel_rpm', 200.0)
        self.declare_parameter('command_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.30)
        self.declare_parameter('telemetry_timeout', 0.50)
        self.declare_parameter('use_imu_yaw_for_odom', True)
        self.declare_parameter('motor_signs', [1, 1, 1, 1])
        self.declare_parameter('encoder_signs', [1, 1, 1, 1])
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'base_link')

        self.port = self.get_parameter('port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.half_length = float(self.get_parameter('half_length').value)
        self.half_width = float(self.get_parameter('half_width').value)
        self.counts_per_revolution = int(
            self.get_parameter('counts_per_revolution').value
        )
        self.max_wheel_rpm = float(self.get_parameter('max_wheel_rpm').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.telemetry_timeout = float(
            self.get_parameter('telemetry_timeout').value
        )
        self.use_imu_yaw_for_odom = bool(
            self.get_parameter('use_imu_yaw_for_odom').value
        )
        self.motor_signs = _signs(
            list(self.get_parameter('motor_signs').value), 'motor_signs'
        )
        self.encoder_signs = _signs(
            list(self.get_parameter('encoder_signs').value), 'encoder_signs'
        )
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value

        if self.counts_per_revolution <= 0:
            raise ValueError('counts_per_revolution must be positive')
        if self.wheel_diameter <= 0.0:
            raise ValueError('wheel_diameter must be positive')
        if self.half_length < 0.0 or self.half_width < 0.0:
            raise ValueError('wheel half dimensions cannot be negative')
        if self.half_length + self.half_width <= 0.0:
            raise ValueError('half_length + half_width must be positive')
        if self.cmd_vel_timeout <= 0.0 or self.telemetry_timeout <= 0.0:
            raise ValueError('command and telemetry timeouts must be positive')

        self.serial_port: Optional[serial.Serial] = None
        self.parser = FrameParser()
        self.next_connect_time = 0.0
        self.last_log_time = -10.0
        self.last_telemetry_time: Optional[float] = None
        self.connection_time: Optional[float] = None
        self.telemetry_timeout_reported = False

        self.latest_twist = Twist()
        self.last_cmd_vel_time: Optional[float] = None
        self.previous_counts: Optional[tuple[int, int, int, int]] = None
        self.previous_mcu_time: Optional[int] = None
        self.previous_imu_yaw: Optional[float] = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.watchdog_reported = False
        self.stall_reported = False
        self.uart_recovery_reported = False
        self.last_body_velocity = (0.0, 0.0, 0.0)

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 20)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 20)
        self.rpm_publisher = self.create_publisher(Float32MultiArray, 'wheel_rpm', 20)
        self.flags_publisher = self.create_publisher(UInt16, 'base/status_flags', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 20)

        command_rate = float(self.get_parameter('command_rate').value)
        if command_rate <= 0.0:
            raise ValueError('command_rate must be positive')
        self.create_timer(0.005, self._serial_io)
        self.create_timer(1.0 / command_rate, self._send_command)
        # 定时发布 odom 到 base_link 的变换，避免停车时 TF 过期。
        self.create_timer(0.05, self._publish_odometry_heartbeat)
        self.create_timer(0.25, self._check_link_health)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _log_connection_error(self, message: str) -> None:
        now = self._now_seconds()
        if now - self.last_log_time >= 2.0:
            self.get_logger().warning(message)
            self.last_log_time = now

    def _connect(self) -> None:
        now = self._now_seconds()
        if now < self.next_connect_time:
            return
        self.next_connect_time = now + 1.0
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0,
                write_timeout=0.05,
                exclusive=True,
            )
        except (OSError, serial.SerialException) as exc:
            self.serial_port = None
            self._log_connection_error(f'Cannot open base serial port {self.port}: {exc}')
            return

        self.parser.reset()
        self.previous_counts = None
        self.previous_mcu_time = None
        self.previous_imu_yaw = None
        self.connection_time = now
        self.last_telemetry_time = None
        self.telemetry_timeout_reported = False
        self.last_body_velocity = (0.0, 0.0, 0.0)
        self.get_logger().info(
            f'Connected to robot base on {self.port} at {self.baud_rate} baud'
        )

    def _disconnect(self, reason: Exception) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except OSError:
                pass
        self.serial_port = None
        self.connection_time = None
        self.last_telemetry_time = None
        self.next_connect_time = self._now_seconds() + 1.0
        self._log_connection_error(f'Robot base serial link lost: {reason}')

    def _serial_io(self) -> None:
        if self.serial_port is None:
            self._connect()
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting <= 0:
                return
            data = self.serial_port.read(min(waiting, 4096))
        except (OSError, serial.SerialException) as exc:
            self._disconnect(exc)
            return

        for frame in self.parser.feed(data):
            if frame.command != CMD_TELEMETRY:
                continue
            try:
                telemetry = decode_telemetry(frame.payload)
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                continue
            self.last_telemetry_time = self._now_seconds()
            self.telemetry_timeout_reported = False
            self._handle_telemetry(telemetry)

    def _check_link_health(self) -> None:
        if self.serial_port is None or self.connection_time is None:
            return
        last_rx = (
            self.last_telemetry_time
            if self.last_telemetry_time is not None
            else self.connection_time
        )
        if self._now_seconds() - last_rx > self.telemetry_timeout:
            # 遥测超时后清零速度，避免继续发布旧速度。
            self.last_body_velocity = (0.0, 0.0, 0.0)
            if not self.telemetry_timeout_reported:
                self.get_logger().error(
                    'Base serial port is open but STM32 telemetry has timed out'
                )
                self.telemetry_timeout_reported = True

    def _cmd_vel_callback(self, message: Twist) -> None:
        self.latest_twist = message
        self.last_cmd_vel_time = self._now_seconds()

    def _send_command(self) -> None:
        if self.serial_port is None:
            return
        now = self._now_seconds()
        stale = (
            self.last_cmd_vel_time is None
            or now - self.last_cmd_vel_time > self.cmd_vel_timeout
        )
        if stale:
            vx = vy = omega = 0.0
        else:
            vx = self.latest_twist.linear.x
            vy = self.latest_twist.linear.y
            omega = self.latest_twist.angular.z

        wheel_rpm = twist_to_wheel_rpm(
            vx,
            vy,
            omega,
            self.wheel_diameter,
            self.half_length,
            self.half_width,
        )
        wheel_rpm = limit_wheel_rpm(wheel_rpm, self.max_wheel_rpm)
        signed_rpm = tuple(
            value * sign for value, sign in zip(wheel_rpm, self.motor_signs)
        )
        try:
            self.serial_port.write(encode_wheel_rpm(signed_rpm))
        except (OSError, serial.SerialException) as exc:
            self._disconnect(exc)

    def _handle_telemetry(self, telemetry: Telemetry) -> None:
        imu_valid = bool(telemetry.flags & STATUS_IMU_VALID)
        current_imu_yaw = (
            math.radians(telemetry.yaw_degrees) if imu_valid else None
        )

        rpm_message = Float32MultiArray()
        rpm_message.data = [
            float(value * sign)
            for value, sign in zip(telemetry.wheel_rpm, self.encoder_signs)
        ]
        self.rpm_publisher.publish(rpm_message)

        flags_message = UInt16()
        flags_message.data = telemetry.flags
        self.flags_publisher.publish(flags_message)
        if telemetry.flags & STATUS_WATCHDOG_STOPPED:
            if not self.watchdog_reported:
                self.get_logger().warning('STM32 command watchdog stopped the motors')
                self.watchdog_reported = True
        else:
            self.watchdog_reported = False

        if telemetry.flags & STATUS_STALL_PROTECTED:
            if not self.stall_reported:
                self.get_logger().error(
                    'STM32 stall protection stopped all motors '
                    f'(event {telemetry.stall_events}); clear the obstacle '
                    'and command zero before moving again'
                )
                self.stall_reported = True
        elif self.stall_reported:
            self.get_logger().info('STM32 stall protection released')
            self.stall_reported = False

        if telemetry.flags & STATUS_UART_RECOVERED:
            if not self.uart_recovery_reported:
                self.get_logger().warning(
                    'STM32 recovered UART8 reception after a serial error'
                )
                self.uart_recovery_reported = True

        if imu_valid:
            self._publish_imu(telemetry)

        counts = tuple(
            value * sign
            for value, sign in zip(telemetry.encoder_counts, self.encoder_signs)
        )
        if self.previous_counts is None or self.previous_mcu_time is None:
            self.previous_counts = counts
            self.previous_mcu_time = telemetry.timestamp_ms
            self.previous_imu_yaw = current_imu_yaw
            return

        elapsed_ms = (telemetry.timestamp_ms - self.previous_mcu_time) & 0xFFFFFFFF
        deltas = tuple(
            _counter_delta(current, previous)
            for current, previous in zip(counts, self.previous_counts)
        )
        self.previous_counts = counts
        self.previous_mcu_time = telemetry.timestamp_ms
        if elapsed_ms == 0 or elapsed_ms > 1000:
            self.last_body_velocity = (0.0, 0.0, 0.0)
            self.previous_imu_yaw = current_imu_yaw
            return

        distance_per_count = (
            math.pi * self.wheel_diameter / self.counts_per_revolution
        )
        wheel_distances = tuple(delta * distance_per_count for delta in deltas)
        dx_body, dy_body, wheel_dyaw = wheel_distances_to_body_delta(
            wheel_distances, self.half_length, self.half_width
        )
        odom_dyaw = wheel_dyaw
        if (
            self.use_imu_yaw_for_odom
            and current_imu_yaw is not None
            and self.previous_imu_yaw is not None
        ):
            odom_dyaw = _normalise_angle(
                current_imu_yaw - self.previous_imu_yaw
            )
        self.previous_imu_yaw = current_imu_yaw

        yaw_midpoint = self.yaw + odom_dyaw * 0.5
        self.x += dx_body * math.cos(yaw_midpoint) - dy_body * math.sin(yaw_midpoint)
        self.y += dx_body * math.sin(yaw_midpoint) + dy_body * math.cos(yaw_midpoint)
        self.yaw = _normalise_angle(self.yaw + odom_dyaw)

        elapsed = elapsed_ms / 1000.0
        self.last_body_velocity = (
            dx_body / elapsed,
            dy_body / elapsed,
            odom_dyaw / elapsed,
        )

    def _publish_odometry_heartbeat(self) -> None:
        """按固定频率发布里程计位姿和 TF。"""
        self._publish_odometry(*self.last_body_velocity)

    def _publish_odometry(self, vx: float, vy: float, omega: float) -> None:
        stamp = self.get_clock().now().to_msg()
        half_yaw = self.yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        odometry.twist.twist.linear.x = vx
        odometry.twist.twist.linear.y = vy
        odometry.twist.twist.angular.z = omega
        odometry.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        odometry.twist.covariance = odometry.pose.covariance
        self.odom_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _publish_imu(self, telemetry: Telemetry) -> None:
        yaw = math.radians(telemetry.yaw_degrees)
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.imu_frame
        message.orientation.z = math.sin(yaw * 0.5)
        message.orientation.w = math.cos(yaw * 0.5)
        message.orientation_covariance = [
            999.0, 0.0, 0.0,
            0.0, 999.0, 0.0,
            0.0, 0.0, 0.05,
        ]
        message.angular_velocity.z = math.radians(telemetry.gyro_rate_dps)
        message.angular_velocity_covariance = [
            999.0, 0.0, 0.0,
            0.0, 999.0, 0.0,
            0.0, 0.0, 0.02,
        ]
        message.linear_acceleration_covariance[0] = -1.0
        self.imu_publisher.publish(message)

    def destroy_node(self) -> bool:
        if self.serial_port is not None:
            try:
                self.serial_port.write(encode_stop())
                self.serial_port.close()
            except (OSError, serial.SerialException):
                pass
            self.serial_port = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
