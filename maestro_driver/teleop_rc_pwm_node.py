"""Keyboard teleop node: publishes rc_pwm based on key input."""

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
Keyboard Teleop (rc_pwm)
------------------------
  q : forward + steer left
  w : forward
  e : forward + steer right
  a : steer left
  d : steer right
  s : backward

  (no key) : neutral (stop)
  Ctrl+C   : quit
"""

NEUTRAL = 1500
OFFSET = 100

# key -> (throttle_offset, steering_offset)
KEY_MAP = {
    'w': (+OFFSET, 0),
    's': (-OFFSET, 0),
    'a': (0, -OFFSET),
    'd': (0, +OFFSET),
    'q': (+OFFSET, -OFFSET),
    'e': (+OFFSET, +OFFSET),
}


class TeleopRcPwmNode(Node):
    """Publish UInt16MultiArray on /rc_pwm from keyboard input."""

    def __init__(self):
        super().__init__('teleop_rc_pwm_node')

        self.declare_parameter('pwm_neutral', NEUTRAL)
        self.declare_parameter('pwm_offset', OFFSET)

        self.pwm_neutral = self.get_parameter('pwm_neutral').value
        self.pwm_offset = self.get_parameter('pwm_offset').value

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
        """Read key and publish rc_pwm."""
        # Drain stdin buffer and keep only the last key
        key = None
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)

        msg = UInt16MultiArray()

        if key is not None and key in KEY_MAP:
            thr_off, steer_off = KEY_MAP[key]
            # Scale offsets by parameter ratio
            scale = self.pwm_offset / OFFSET
            ch1 = self.pwm_neutral + int(thr_off * scale)
            ch2 = self.pwm_neutral + int(steer_off * scale)
        else:
            ch1 = self.pwm_neutral
            ch2 = self.pwm_neutral

        msg.data = [ch1, ch2]
        self.pub.publish(msg)

    def destroy_node(self):
        """Restore terminal settings."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    """Entry point for teleop_rc_pwm_node."""
    rclpy.init(args=args)
    node = TeleopRcPwmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send neutral before shutting down
        neutral_msg = UInt16MultiArray()
        neutral_msg.data = [node.pwm_neutral, node.pwm_neutral]
        node.pub.publish(neutral_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
