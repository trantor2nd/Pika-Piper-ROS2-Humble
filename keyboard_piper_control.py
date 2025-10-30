#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys, termios, tty, time, threading

# ========== 配置部分 ==========
JOINT_LIMITS = [
    (-3.14,  3.14),  # joint1
    ( 0.00,  1.57),  # joint2
    (-2.00,  0.00),  # joint3
    (-1.50,  1.80),  # joint4
    (-1.30,  1.57),  # joint5
    (-3.14,  3.14),  # joint6
]

HOME_POS = [0.0, -0.035, 0.0, 0.0, 0.35, 0.0]
STEP = 0.1


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub = self.create_publisher(JointState, '/joint_ctrl_single', 10)
        self.sub = self.create_subscription(JointState, '/joint_states_single', self.feedback_cb, 10)
        self.joint_positions = [0.0] * 6
        self.actual_positions = [0.0] * 6

        s = STEP
        self.key_map = {
            'q': (0, +s), 'a': (0, -s),
            'w': (1, +s), 's': (1, -s),
            'e': (2, +s), 'd': (2, -s),
            'r': (3, +s), 'f': (3, -s),
            't': (4, +s), 'g': (4, -s),
            'y': (5, +s), 'h': (5, -s),
        }

    def feedback_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            self.actual_positions = list(msg.position[:6])

    def publish_joint_state(self):
        msg = JointState()
        msg.name = [f"joint{i+1}" for i in range(6)]
        msg.position = self.joint_positions
        self.pub.publish(msg)

    def print_both(self, prefix=""):
        cmd = [f"{x:.2f}" for x in self.joint_positions]
        fbk = [f"{x:.2f}" for x in self.actual_positions]
        print(f"[FBK]{fbk} | {prefix}[CMD]{cmd}", flush=True)

    def go_home(self, sec: float = 3.0, steps: int = 30):
        print("[INFO] 回到 HOME ...", flush=True)
        start = self.joint_positions[:]
        target = [clamp(v, *JOINT_LIMITS[i]) for i, v in enumerate(HOME_POS)]
        for i in range(1, steps + 1):
            r = i / steps
            self.joint_positions = [s + (t - s) * r for s, t in zip(start, target)]
            self.publish_joint_state()
            time.sleep(sec / steps)
        self.joint_positions = target
        self.publish_joint_state()
        self.print_both(prefix="[HOME] ")

    def run(self):
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()

        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        print("键盘控制：q/a w/s e/d r/f t/g y/h 调关节 | Z=回HOME | 空格=退出", flush=True)
        try:
            while True:
                key = sys.stdin.read(1)
                if key == ' ':
                    break
                if key in ('z', 'Z'):
                    self.go_home()
                    continue
                if key in self.key_map:
                    idx, delta = self.key_map[key]
                    lo, hi = JOINT_LIMITS[idx]
                    self.joint_positions[idx] = clamp(self.joint_positions[idx] + delta, lo, hi)
                    self.publish_joint_state()
                    self.print_both()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.go_home()


def main():
    rclpy.init()
    node = TeleopNode()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
