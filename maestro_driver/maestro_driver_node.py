"""ROS2 bridge node for Pololu Maestro servo controller.

Subscribes to /rc_pwm (UInt16MultiArray) and sends RC PWM values
to a Pololu Maestro servo controller over USB serial using the
Compact Protocol Set Target command.
"""

import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import serial
from std_msgs.msg import UInt16MultiArray


class MaestroDriverNode(Node):
    """Bridge node that forwards RC PWM commands to a Pololu Maestro controller.

    Subscribes to /rc_pwm topic and translates PWM pulse widths (microseconds)
    into Maestro Compact Protocol Set Target commands sent over USB serial.
    Includes failsafe functionality to send neutral PWM when communication
    with the upstream controller is lost.
    """

    def __init__(self) -> None:
        super().__init__('maestro_driver_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('ch1_channel', 0)
        self.declare_parameter('ch2_channel', 1)
        self.declare_parameter('failsafe_timeout', 0.5)
        self.declare_parameter('failsafe_pwm', 1500)
        self.declare_parameter('pwm_min', 1000)
        self.declare_parameter('pwm_max', 2000)

        # Read parameters
        self.serial_port: str = self.get_parameter('serial_port').value
        self.ch1_channel: int = self.get_parameter('ch1_channel').value
        self.ch2_channel: int = self.get_parameter('ch2_channel').value
        self.failsafe_timeout: float = self.get_parameter('failsafe_timeout').value
        self.failsafe_pwm: int = self.get_parameter('failsafe_pwm').value
        self.pwm_min: int = self.get_parameter('pwm_min').value
        self.pwm_max: int = self.get_parameter('pwm_max').value

        # State variables
        self.ser: serial.Serial | None = None
        self.last_msg_time: rclpy.time.Time | None = None
        self.is_failsafe: bool = True
        self._reconnect_timer: rclpy.timer.Timer | None = None

        # Attempt initial serial connection
        self._connect_serial()

        # Subscriber with BestEffort QoS, depth=1
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            '/rc_pwm',
            self._rc_pwm_callback,
            qos,
        )

        # Failsafe check timer
        self.failsafe_timer = self.create_timer(
            self.failsafe_timeout, self._failsafe_check
        )

        self.get_logger().info(
            f'Maestro driver started (port={self.serial_port}, '
            f'ch1={self.ch1_channel}, ch2={self.ch2_channel})'
        )

    def _rc_pwm_callback(self, msg: UInt16MultiArray) -> None:
        """Handle incoming RC PWM messages.

        Clamps values to [pwm_min, pwm_max] and sends Set Target commands
        to the Maestro for both channels.
        """
        if len(msg.data) < 2:
            self.get_logger().warn(
                'Received UInt16MultiArray with less than 2 elements, ignoring',
                throttle_duration_sec=1.0,
            )
            return

        if self.ser is None:
            return

        ch1_pwm = self._clamp(int(msg.data[0]))
        ch2_pwm = self._clamp(int(msg.data[1]))

        self._send_target(self.ch1_channel, ch1_pwm)
        self._send_target(self.ch2_channel, ch2_pwm)

        self.last_msg_time = self.get_clock().now()

        # Recover from failsafe
        if self.is_failsafe:
            self.is_failsafe = False
            self.get_logger().info('Recovered from failsafe, resuming normal operation')

    def _failsafe_check(self) -> None:
        """Timer callback to check for topic timeout and trigger failsafe."""
        if self.last_msg_time is None:
            # No message received yet since startup
            self._send_failsafe()
            return

        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > self.failsafe_timeout:
            if not self.is_failsafe:
                self.is_failsafe = True
            self.get_logger().warn(
                f'Failsafe activated: no /rc_pwm for {elapsed:.2f}s',
                throttle_duration_sec=1.0,
            )
            self._send_failsafe()

    def _send_target(self, channel: int, pwm_us: int) -> None:
        """Send a Set Target command to the Maestro.

        Converts PWM pulse width in microseconds to quarter-microseconds
        and sends the Compact Protocol Set Target command (0x84).

        Args:
            channel: Maestro channel number.
            pwm_us: PWM pulse width in microseconds.
        """
        if self.ser is None:
            return

        target = pwm_us * 4  # quarter-microseconds
        target_low = target & 0x7F
        target_high = (target >> 7) & 0x7F

        cmd = struct.pack('BBBB', 0x84, channel, target_low, target_high)

        try:
            self.ser.write(cmd)
        except serial.SerialException as e:
            self.get_logger().error(
                f'Serial write error: {e}', throttle_duration_sec=1.0
            )
            self._close_and_reconnect()

    def _send_failsafe(self) -> None:
        """Send failsafe PWM value to both channels."""
        self._send_target(self.ch1_channel, self.failsafe_pwm)
        self._send_target(self.ch2_channel, self.failsafe_pwm)

    def _connect_serial(self) -> None:
        """Attempt to open the serial port.

        On failure, starts a reconnection timer to retry periodically.
        """
        try:
            self.ser = serial.Serial(self.serial_port)
            self.get_logger().info(f'Connected to {self.serial_port}')
            # Cancel reconnect timer if running
            if self._reconnect_timer is not None:
                self._reconnect_timer.cancel()
                self.destroy_timer(self._reconnect_timer)
                self._reconnect_timer = None
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().error(
                f'Failed to open {self.serial_port}: {e}',
                throttle_duration_sec=2.0,
            )
            self._start_reconnect_timer()

    def _close_and_reconnect(self) -> None:
        """Close the current serial port and begin reconnection attempts."""
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self._start_reconnect_timer()

    def _start_reconnect_timer(self) -> None:
        """Start a timer to periodically retry serial connection."""
        if self._reconnect_timer is not None:
            return
        self._reconnect_timer = self.create_timer(1.0, self._reconnect_callback)

    def _reconnect_callback(self) -> None:
        """Timer callback to retry serial connection."""
        self.get_logger().info(
            f'Attempting to reconnect to {self.serial_port}...',
            throttle_duration_sec=5.0,
        )
        self._connect_serial()

    def _clamp(self, value: int) -> int:
        """Clamp a PWM value to [pwm_min, pwm_max].

        Args:
            value: Raw PWM value in microseconds.

        Returns:
            Clamped PWM value.
        """
        return max(self.pwm_min, min(self.pwm_max, value))

    def on_shutdown(self) -> None:
        """Send failsafe values and close the serial port on shutdown."""
        self.get_logger().info('Shutting down, sending failsafe...')
        self._send_failsafe()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


def main(args: list[str] | None = None) -> None:
    """Entry point for the maestro_driver_node."""
    rclpy.init(args=args)
    node = MaestroDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
