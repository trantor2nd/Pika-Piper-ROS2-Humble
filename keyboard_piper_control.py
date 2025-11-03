#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import logging
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from pika.gripper import Gripper

# ========== 配置部分 ==========
JOINT_LIMITS = [
    (-3.14,  3.14),
    ( 0.00,  2.00),
    (-2.00,  0.00),
    (-1.50,  1.80),
    (-1.30,  1.57),
    (-3.14,  3.14),
]

HOME_POS = [0.0, -0.035, 0.0, 0.0, 0.35, 0.0]
STEP = 0.05

GRIPPER_PORT = "/dev/ttyUSB1"   # 串口路径
GRIPPER_MIN = 0.0               # 完全闭合（mm）
GRIPPER_MAX = 90.0              # 完全张开（mm）
GRIPPER_STEP = 20             # 每次按键增减 20mm


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        logging.getLogger("pika.serial_comm").setLevel(logging.ERROR)
        self.pub = self.create_publisher(JointState, '/joint_ctrl_single', 10)
        self.pub_record = self.create_publisher(String, '/record_cmd', 10)
        self.pub_gripper_state = self.create_publisher(Float64, '/gripper_state', 10)
        self.pub_enable = self.create_publisher(Bool, '/enable_flag', 10)
        self.sub = self.create_subscription(JointState, '/joint_states_single', self.feedback_cb, 10)
        self.joint_positions = [0.0] * 6
        self.actual_positions = [0.0] * 6
        self.recording = False
        self._running = True

        s = STEP
        self.key_map = {
            'a': (0, +s), 'd': (0, -s),
            'w': (1, +s), 's': (1, -s),
            'f': (2, +s), 'r': (2, -s),
            't': (3, +s), 'g': (3, -s),
            'y': (4, +s), 'h': (4, -s),
            'e': (5, +s), 'q': (5, -s),
        }

        # === 初始化夹爪 ===
        self.gripper = None
        self.gripper_pos = 0.0
        self._init_gripper()

    # ==============================
    # 初始化夹爪
    # ==============================
    def _init_gripper(self):
        try:
            print(f"[INFO] 尝试连接夹爪 {GRIPPER_PORT} ...")
            self.gripper = Gripper(GRIPPER_PORT)
            if not self.gripper.connect():
                print("[WARN] 无法连接夹爪，请检查连接")
                self.gripper = None
                return
            if not self.gripper.enable():
                print("[WARN] 启动夹爪失败")
                self.gripper = None
                return
            print("[OK] 夹爪连接成功 ✅")
            self.gripper_pos = self.gripper.get_gripper_distance()
            self._publish_gripper_state()
            self.pub_enable.publish(Bool(data=True))
        except Exception as e:
            print(f"[ERROR] 初始化夹爪失败: {e}")
            self.gripper = None

    # ==============================
    # 夹爪控制
    # ==============================
    def _publish_gripper_state(self):
        self.pub_gripper_state.publish(Float64(data=float(self.gripper_pos)))

    def gripper_step_open(self):
        if self.gripper:
            self.gripper_pos = clamp(self.gripper_pos + GRIPPER_STEP, GRIPPER_MIN, GRIPPER_MAX)
            #print(f"[CMD] 张开夹爪 -> {self.gripper_pos:.1f} mm")
            self.gripper.set_gripper_distance(self.gripper_pos)
            self._publish_gripper_state()

    def gripper_step_close(self):
        if self.gripper:
            self.gripper_pos = clamp(self.gripper_pos - GRIPPER_STEP, GRIPPER_MIN, GRIPPER_MAX)
            #print(f"[CMD] 闭合夹爪 -> {self.gripper_pos:.1f} mm")
            self.gripper.set_gripper_distance(self.gripper_pos)
            self._publish_gripper_state()

    def gripper_disable(self):
        if self.gripper:
            try:
                print("[INFO] 下电夹爪 ...")
                self.gripper.disable()
                self.gripper.disconnect()
                print("[OK] 夹爪已关闭")
            except Exception as e:
                print(f"[WARN] 关闭夹爪失败: {e}")

    # ==============================
    # 录制控制
    # ==============================
    def toggle_recording(self, start: bool):
        if start and not self.recording:
            self.pub_record.publish(String(data="start"))
            self.recording = True
            print("[REC] 开始采集")
        elif not start and self.recording:
            self.pub_record.publish(String(data="stop"))
            self.recording = False
            print("[REC] 停止采集")

    # ==============================
    # 机械臂控制
    # ==============================
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
        #print(f"[FBK]{fbk} | {prefix}[CMD]{cmd}", flush=True)

    def go_home(self, sec: float = 3.0, steps: int = 30):
        print("[INFO] 回 HOME ...")
        start = self.joint_positions[:]
        target = [clamp(v, *JOINT_LIMITS[i]) for i, v in enumerate(HOME_POS)]
        for i in range(1, steps + 1):
            r = i / steps
            self.joint_positions = [s + (t - s) * r for s, t in zip(start, target)]
            self.publish_joint_state()
            time.sleep(sec / steps)
        self.joint_positions = target
        self.publish_joint_state()
        self.print_both("[HOME] ")

    # ==============================
    # 主循环
    # ==============================
    def run(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)

        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        sys.stdin.flush()

        print("键盘控制：")
        print("a/d w/s r/f t/g q/e y/h q/e 控制关节") 
        print("u 张开夹爪 | j 闭合夹爪")
        print("o 开始录制 | p 停止录制")
        print("z 回HOME | 空格退出")
        print("====================================", flush=True)

        try:
            while self._running and rclpy.ok():
                executor.spin_once(timeout_sec=0.0)
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not rlist:
                    continue

                key = sys.stdin.read(1)
                if key == ' ':
                    print("[INFO] 空格退出程序")
                    self.toggle_recording(False)
                    self._running = False
                elif key in ('z', 'Z'):
                    self.go_home()
                elif key in ('j', 'J'):
                    self.gripper_step_open()
                elif key in ('u', 'U'):
                    self.gripper_step_close()
                elif key in ('o', 'O'):
                    self.toggle_recording(True)
                elif key in ('p', 'P'):
                    self.toggle_recording(False)
                elif key in self.key_map:
                    idx, delta = self.key_map[key]
                    lo, hi = JOINT_LIMITS[idx]
                    self.joint_positions[idx] = clamp(self.joint_positions[idx] + delta, lo, hi)
                    self.publish_joint_state()
                    self.print_both()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.go_home()
            self.pub_record.publish(String(data="exit"))
            if self.gripper:
                try:
                    self.gripper_pos = self.gripper.get_gripper_distance()
                except Exception:
                    pass
                self._publish_gripper_state()
            self.pub_enable.publish(Bool(data=False))
            self.gripper_disable()
            executor.remove_node(self)
            executor.shutdown()


def main():
    rclpy.init()
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
