#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Trajectory replay utility.
Load a recorded episode (joint + gripper state) and command the robot
to follow the same trajectory at a fixed rate.
"""

import argparse
import glob
import logging
import os
import time
from typing import List, Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    from piper_msgs.srv import Enable
except ImportError:  # pragma: no cover - service type may not be available in pure replay env
    Enable = None  # type: ignore

from pika.gripper import Gripper


# ----------------------------- Configuration -----------------------------
JOINT_LIMITS = [
    (-3.14, 3.14),
    (0.00, 2.00),
    (-2.00, 0.00),
    (-1.50, 1.80),
    (-1.30, 1.57),
    (-3.14, 3.14),
]

HOME_POS = [0.0, -0.035, 0.0, 0.0, 0.35, 0.0]
GRIPPER_MIN = 0.0      # mm
GRIPPER_MAX = 90.0     # mm
DEFAULT_RATE = 5.0     # Hz
DEFAULT_WARMUP = 4.0   # s (move to first waypoint)
GRIPPER_PORT = "/dev/ttyUSB1"


def clamp(value: float, bounds: Sequence[float]) -> float:
    lo, hi = bounds
    return max(lo, min(hi, value))


class TrajectoryPlayer(Node):
    """ROS2 node that replays state logs as joint commands."""

    def __init__(
        self,
        states: List[List[float]],
        rate_hz: float,
        warmup_time: float,
        loop: bool,
        gripper_port: str,
    ) -> None:
        super().__init__("trajectory_player")
        self.pub_joint = self.create_publisher(JointState, "/joint_ctrl_single", 10)
        self.sub_joint = self.create_subscription(
            JointState, "/joint_states_single", self.feedback_cb, 10
        )
        self.pub_enable = self.create_publisher(Bool, "/enable_flag", 10)
        self.enable_client = (
            self.create_client(Enable, "/enable_srv") if Enable is not None else None
        )

        self.states = states
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz if rate_hz > 0 else 0.1
        self.loop = loop
        self.warmup_time = warmup_time
        self.timer = None

        self.actual_positions = [0.0] * 6
        self.command_index = 0

        self.gripper = None
        self.gripper_port = gripper_port
        self._init_gripper()

    # --------------------------- Initialisation ---------------------------
    def _init_gripper(self) -> None:
        try:
            self.get_logger().info(f"连接夹爪 {self.gripper_port} ...")
            self.gripper = Gripper(self.gripper_port)
            if not self.gripper.connect():
                self.get_logger().warning("夹爪连接失败，将跳过夹爪复现")
                self.gripper = None
                return
            if not self.gripper.enable():
                self.get_logger().warning("夹爪上电失败，将跳过夹爪复现")
                self.gripper.disconnect()
                self.gripper = None
                return
            current = clamp(
                self.gripper.get_gripper_distance(), (GRIPPER_MIN, GRIPPER_MAX)
            )
            self.get_logger().info(f"夹爪连接成功，当前距离 {current:.2f} mm")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"初始化夹爪异常: {exc}")
            self.gripper = None

    def prepare(self) -> None:
        """Move to the first frame and start timer."""
        if not self.states:
            self.get_logger().warning("未找到任何状态数据，取消复现")
            return

        self._set_enable(True)

        first = self.states[0]
        self.get_logger().info("移动到轨迹起点 ...")
        self._move_linearly(HOME_POS + [GRIPPER_MIN], first, duration=self.warmup_time)

        loop_suffix = " (循环播放)" if self.loop else ""
        self.get_logger().info(
            f"开始轨迹复现，共 {len(self.states)} 帧，频率 {self.rate_hz:.2f} Hz{loop_suffix}"
        )
        self.timer = self.create_timer(self.dt, self._step)

    # ----------------------------- Playback ------------------------------
    def _step(self) -> None:
        if not self.states:
            return

        if self.command_index >= len(self.states):
            if self.loop:
                self.command_index = 0
                self.get_logger().info("轨迹循环重新开始")
            else:
                self.get_logger().info("轨迹复现完成")
                if self.timer:
                    self.timer.cancel()
                    self.timer = None
                # After final frame, trigger shutdown
                self._on_playback_complete()
                return

        target = self.states[self.command_index]
        self._send_command(target)
        self.command_index += 1

    def _send_command(self, target: Sequence[float]) -> None:
        joint_cmd = [
            clamp(target[i], JOINT_LIMITS[i])
            for i in range(min(6, len(target)))
        ]
        msg = JointState()
        msg.name = [f"joint{i+1}" for i in range(len(joint_cmd))]
        msg.position = joint_cmd
        self.pub_joint.publish(msg)

        if self.gripper and len(target) >= 7:
            distance = clamp(target[6], (GRIPPER_MIN, GRIPPER_MAX))
            try:
                self.gripper.set_gripper_distance(distance)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"设置夹爪失败: {exc}")

    def _move_linearly(
        self, start: Sequence[float], end: Sequence[float], duration: float, steps: int = 60
    ) -> None:
        duration = max(duration, 1e-3)
        steps = max(steps, 1)
        for i in range(1, steps + 1):
            ratio = i / steps
            interp = [
                s + (e - s) * ratio
                for s, e in zip(start, end)
            ]
            self._send_command(interp)
            time.sleep(duration / steps)

    # --------------------------- Feedback / Exit -------------------------
    def feedback_cb(self, msg: JointState) -> None:
        count = min(len(msg.position), len(self.actual_positions))
        for i in range(count):
            self.actual_positions[i] = msg.position[i]

    def _on_playback_complete(self) -> None:
        self.get_logger().info("等待退出信号 ...")
        # stop spinning after a short delay to flush last messages
        rclpy.shutdown()

    def shutdown(self) -> None:
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.get_logger().info("轨迹复现退出，返回 HOME")
        try:
            self._move_linearly(
                self.states[min(self.command_index, len(self.states) - 1)]
                if self.states
                else HOME_POS + [GRIPPER_MIN],
                HOME_POS + [GRIPPER_MIN],
                duration=3.0,
            )
        except Exception:
            pass

        if self.gripper:
            try:
                self.get_logger().info("断电夹爪 ...")
                self.gripper.disable()
                self.gripper.disconnect()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"断开夹爪失败: {exc}")
            self.gripper = None

        self._set_enable(False)

    def _set_enable(self, value: bool) -> None:
        self.pub_enable.publish(Bool(data=value))
        if not self.enable_client:
            return
        if not self.enable_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("enable_srv 不可达，跳过服务调用")
            return
        request = Enable.Request()
        request.enable_request = value
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.cancelled():
            self.get_logger().warning("enable_srv 调用被取消")
        elif future.exception() is not None:
            self.get_logger().warning(f"enable_srv 调用失败: {future.exception()}")
        else:
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"enable_srv 结果异常: {exc}")
            else:
                self.get_logger().info(f"enable_srv 响应: {response}")


# ------------------------------- Utilities -------------------------------
def load_episode(episode_dir: str) -> List[List[float]]:
    state_dir = os.path.join(episode_dir, "state")
    if not os.path.isdir(state_dir):
        raise FileNotFoundError(f"未找到状态目录: {state_dir}")

    files = sorted(glob.glob(os.path.join(state_dir, "*.log")))
    states: List[List[float]] = []
    for path in files:
        try:
            with open(path, "r", encoding="utf-8") as f:
                line = f.readline().strip()
            if not line:
                continue
            values = [float(x) for x in line.split()]
            if len(values) < 7:
                continue
            states.append(values[:7])
        except Exception as exc:  # noqa: BLE001
            logging.getLogger("trajectory_loader").warning(
                "读取 %s 失败: %s", path, exc
            )
    return states


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Piper 轨迹复现")
    parser.add_argument(
        "--dataset-root",
        default="/home/data/Dataset/oracle_dataset_raw",
        help="数据集根目录 (默认: %(default)s)",
    )
    parser.add_argument(
        "--episode",
        required=True,
        help="要复现的 episode 名称 (如 episode_00005)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE,
        help="复现频率 Hz (默认: %(default)s)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=DEFAULT_WARMUP,
        help="上线时从 HOME 到首帧的过渡时间，单位秒 (默认: %(default)s)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="循环播放轨迹",
    )
    parser.add_argument(
        "--gripper-port",
        default=GRIPPER_PORT,
        help=f"夹爪串口 (默认: {GRIPPER_PORT})",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    episode_path = os.path.join(args.dataset_root, args.episode)

    if not os.path.isdir(episode_path):
        raise FileNotFoundError(f"未找到 episode 目录: {episode_path}")

    states = load_episode(episode_path)
    if not states:
        raise RuntimeError(f"{args.episode} 中无有效状态数据")

    rclpy.init()
    node = TrajectoryPlayer(
        states=states,
        rate_hz=args.rate,
        warmup_time=args.warmup,
        loop=args.loop,
        gripper_port=args.gripper_port,
    )

    try:
        node.prepare()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断轨迹复现")
    finally:
        try:
            node.shutdown()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
