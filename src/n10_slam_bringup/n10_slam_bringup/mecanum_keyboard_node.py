import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MOVE_BINDINGS = {
    'i': (1.0, 0.0, 0.0),
    ',': (-1.0, 0.0, 0.0),
    'j': (0.0, 0.0, 1.0),
    'l': (0.0, 0.0, -1.0),
    'I': (1.0, 0.0, 0.0),
    '<': (-1.0, 0.0, 0.0),
    'J': (0.0, 1.0, 0.0),
    'L': (0.0, -1.0, 0.0),
    'U': (1.0, 1.0, 0.0),
    'O': (1.0, -1.0, 0.0),
    'M': (-1.0, 1.0, 0.0),
    '>': (-1.0, -1.0, 0.0),
}

COMMAND_NAMES = {
    'i': 'forward',
    ',': 'backward',
    'j': 'rotate-left',
    'l': 'rotate-right',
    'I': 'forward',
    '<': 'backward',
    'J': 'strafe-left',
    'L': 'strafe-right',
    'U': 'forward-left',
    'O': 'forward-right',
    'M': 'backward-left',
    '>': 'backward-right',
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

HELP = """
Mecanum keyboard control (commands are repeated continuously at 20 Hz)

   U    I    O        Shift+J/L: strafe left/right
   J    K    L        i / ,    : forward/backward
   M    <    >        j / l    : rotate left/right

k or Space: stop
q/z: increase/decrease all speeds
w/x: increase/decrease linear speed
e/c: increase/decrease angular speed
Ctrl-C: stop and exit
"""


class MecanumKeyboard(Node):
    def __init__(self):
        super().__init__('mecanum_keyboard')
        self.speed = float(self.declare_parameter('speed', 0.10).value)
        self.turn = float(self.declare_parameter('turn', 0.35).value)
        self.max_speed = float(self.declare_parameter('max_speed', 0.25).value)
        self.max_turn = float(self.declare_parameter('max_turn', 1.0).value)
        self.publish_rate = float(
            self.declare_parameter('publish_rate', 20.0).value
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def handle_key(self, key):
        if key in MOVE_BINDINGS:
            self.x, self.y, self.yaw = MOVE_BINDINGS[key]
            self.show_command(COMMAND_NAMES[key])
        elif key in SPEED_BINDINGS:
            linear_scale, angular_scale = SPEED_BINDINGS[key]
            self.speed = min(self.speed * linear_scale, self.max_speed)
            self.turn = min(self.turn * angular_scale, self.max_turn)
            print(f'speed={self.speed:.3f} m/s turn={self.turn:.3f} rad/s')
        else:
            self.stop()
            self.show_command('stop')

    def show_command(self, name):
        sys.stdout.write(
            f'\rcommand={name:<16} '
            f'vx={self.x * self.speed:+.3f} '
            f'vy={self.y * self.speed:+.3f} '
            f'wz={self.yaw * self.turn:+.3f}   '
        )
        sys.stdout.flush()

    def stop(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def publish_command(self):
        message = Twist()
        message.linear.x = self.x * self.speed
        message.linear.y = self.y * self.speed
        message.angular.z = self.yaw * self.turn
        self.publisher.publish(message)


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('mecanum_keyboard must run in an interactive terminal')

    rclpy.init(args=args)
    node = MecanumKeyboard()
    old_settings = termios.tcgetattr(sys.stdin)
    period = 1.0 / max(node.publish_rate, 1.0)

    print(HELP)
    print(f'speed={node.speed:.3f} m/s turn={node.turn:.3f} rad/s')

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], period)
            if readable:
                key = sys.stdin.read(1)
                if key == '\x03':
                    break
                node.handle_key(key)
            node.publish_command()
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.stop()
        for _ in range(3):
            node.publish_command()
            time.sleep(0.02)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
