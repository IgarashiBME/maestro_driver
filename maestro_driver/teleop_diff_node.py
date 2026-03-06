"""Differential-drive keyboard teleop node: publishes rc_pwm [ch1, ch2, ch3]."""

import atexit
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt16MultiArray

USAGE = """
Keyboard Teleop — Differential Drive (rc_pwm)
----------------------------------------------
  q : forward + steer left
  w : forward
  e : forward + steer right
  a : pivot left
  d : pivot right
  s : backward

  r : rudder (ch3) +step
  f : rudder (ch3) -step
  v : rudder (ch3) reset to neutral

  (no key) : ch1/ch2 neutral (ch3 holds)
  Ctrl+C   : quit
"""


class TeleopDiffNode(Node):
    """Publish UInt16MultiArray on /rc_pwm with differential mixing."""

    def __init__(self):
        super().__init__('teleop_diff_node')

        self.declare_parameter('pwm_center', 1500)
        self.declare_parameter('pwm_offset', 100)
        self.declare_parameter('pwm_min', 1000)
        self.declare_parameter('pwm_max', 2000)
        self.declare_parameter('steering_reverse', 0.0)
        self.declare_parameter('throttle_reverse', 0.0)
        self.declare_parameter('rudder_center', 1500)
        self.declare_parameter('rudder_step', 50)
        self.declare_parameter('rudder_min', 1000)
        self.declare_parameter('rudder_max', 2000)
        self.declare_parameter('rudder_reverse', 0.0)

        self.ch3_pwm = self.get_parameter('rudder_center').value

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(UInt16MultiArray, '/rc_pwm', qos)
        self.timer = self.create_timer(0.1, self._timer_cb)  # 10 Hz

        # Save and switch terminal to raw mode
        self.old_settings = termios.tcgetattr(sys.stdin)
        atexit.register(
            termios.tcsetattr, sys.stdin, termios.TCSANOW, self.old_settings
        )
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info(USAGE)

    def _timer_cb(self):
        """Read key and publish rc_pwm with differential mixing."""
        # Drain stdin buffer and keep only the last key
        key = None
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)

        center = self.get_parameter('pwm_center').value
        offset = self.get_parameter('pwm_offset').value
        pwm_min = self.get_parameter('pwm_min').value
        pwm_max = self.get_parameter('pwm_max').value

        throttle_sign = (
            -1 if self.get_parameter('throttle_reverse').value >= 0.5 else 1
        )
        steer_sign = (
            -1 if self.get_parameter('steering_reverse').value >= 0.5 else 1
        )

        # Determine throttle and steering offsets from key
        throttle = 0
        steering = 0
        if key == 'w':
            throttle = offset
        elif key == 's':
            throttle = -offset
        elif key == 'a':
            steering = -offset
        elif key == 'd':
            steering = offset
        elif key == 'q':
            throttle = offset
            steering = -offset
        elif key == 'e':
            throttle = offset
            steering = offset

        # Handle rudder keys (toggle)
        rudder_step = self.get_parameter('rudder_step').value
        rudder_min = self.get_parameter('rudder_min').value
        rudder_max = self.get_parameter('rudder_max').value
        rudder_sign = (
            -1 if self.get_parameter('rudder_reverse').value >= 0.5 else 1
        )

        if key == 'r':
            self.ch3_pwm += rudder_sign * rudder_step
        elif key == 'f':
            self.ch3_pwm -= rudder_sign * rudder_step
        elif key == 'v':
            self.ch3_pwm = self.get_parameter('rudder_center').value

        self.ch3_pwm = max(rudder_min, min(rudder_max, self.ch3_pwm))

        # Differential mixing: ch1=left, ch2=right
        ch1 = center + throttle_sign * throttle - steer_sign * steering
        ch2 = center + throttle_sign * throttle + steer_sign * steering

        ch1 = max(pwm_min, min(pwm_max, ch1))
        ch2 = max(pwm_min, min(pwm_max, ch2))

        msg = UInt16MultiArray()
        msg.data = [ch1, ch2, self.ch3_pwm]
        self.pub.publish(msg)

    def destroy_node(self):
        """Restore terminal settings."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    """Entry point for teleop_diff_node."""
    rclpy.init(args=args)
    node = TeleopDiffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send neutral before shutting down
        neutral_msg = UInt16MultiArray()
        center = node.get_parameter('pwm_center').value
        rudder_center = node.get_parameter('rudder_center').value
        neutral_msg.data = [center, center, rudder_center]
        node.pub.publish(neutral_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
