#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import time
import threading
import logging
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from pika.camera.fisheye import FisheyeCamera
from pika.camera.realsense import RealSenseCamera


# ==============================
# 配置部分
# ==============================
SAVE_ROOT = "/home/data/Dataset/oracle_dataset_raw"
CAPTURE_HZ = 10.0  # 采样频率 10Hz

# Camera configuration
FISHEYE_RESOLUTION = (640, 480, 30)
REALSENSE_RESOLUTION = (640, 480, 30)
FISHEYE_INDEX = 6
REALSENSE_SN = "230322275684"


def ensure_dir(path: str) -> None:
    if not os.path.exists(path):
        os.makedirs(path)


class DataRecorder(Node):
    def __init__(self) -> None:
        super().__init__("data_recorder")
        logging.getLogger("pika.serial_comm").setLevel(logging.ERROR)
        logging.getLogger("pika.camera.realsense").setLevel(logging.ERROR)

        # ---- ROS 通信 ----
        self.create_subscription(JointState, "/joint_states_single", self.joint_cb, 10)
        self.create_subscription(Float64, "/gripper_state", self.gripper_cb, 10)
        self.create_subscription(String, "/record_cmd", self.cmd_cb, 10)

        # ---- 状态变量 ----
        self.recording = False
        self.running = True
        self.joint_state = [0.0] * 6
        self.gripper_pos = 0.0
        self.episode_dir = None
        self.frame_id = 0

        # ---- 相机实例 ----
        self.fisheye = None
        self.realsense = None
        self._connect_cameras()

        # ---- 启动采集线程 ----
        threading.Thread(target=self._capture_loop, daemon=True).start()
        self.get_logger().info("数据采集节点启动完成 ✅")

    # ==============================
    # 订阅回调
    # ==============================
    def joint_cb(self, msg: JointState) -> None:
        if len(msg.position) >= 6:
            self.joint_state = list(msg.position[:6])

    def gripper_cb(self, msg: Float64) -> None:
        self.gripper_pos = msg.data

    def cmd_cb(self, msg: String) -> None:
        if msg.data == "start":
            if not self.recording:
                self.start_recording()
        elif msg.data == "stop":
            if self.recording:
                self.stop_recording()
        elif msg.data == "exit":
            self.get_logger().info("收到退出信号，准备关闭采集节点")
            self.running = False
            if self.recording:
                self.stop_recording()
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    # ==============================
    # 录制控制
    # ==============================
    def start_recording(self) -> None:
        ensure_dir(SAVE_ROOT)
        episodes = sorted(d for d in os.listdir(SAVE_ROOT) if d.startswith("episode_"))
        next_id = len(episodes)
        self.episode_dir = os.path.join(SAVE_ROOT, f"episode_{next_id:05d}")
        ensure_dir(self.episode_dir)
        ensure_dir(os.path.join(self.episode_dir, "img/fisheye_rgb"))
        ensure_dir(os.path.join(self.episode_dir, "img/realsense_rgb"))
        ensure_dir(os.path.join(self.episode_dir, "state"))

        self.frame_id = 0
        self.recording = True
        self.get_logger().info(f"开始录制 → {self.episode_dir}")

    def stop_recording(self) -> None:
        self.recording = False
        self.get_logger().info(f"停止录制 → {self.episode_dir}")
        self.episode_dir = None

    # ==============================
    # 相机连接
    # ==============================
    def _connect_cameras(self) -> None:
        self.fisheye = FisheyeCamera(
            camera_width=FISHEYE_RESOLUTION[0],
            camera_height=FISHEYE_RESOLUTION[1],
            camera_fps=FISHEYE_RESOLUTION[2],
            device_id=FISHEYE_INDEX,
            fisheye_thread_fps=60,
        )
        if not self.fisheye.connect():
            self.get_logger().error("鱼眼相机连接失败，录制中将缺失鱼眼图像")
            self.fisheye = None

        self.realsense = RealSenseCamera(
            camera_width=REALSENSE_RESOLUTION[0],
            camera_height=REALSENSE_RESOLUTION[1],
            camera_fps=REALSENSE_RESOLUTION[2],
            serial_number=REALSENSE_SN,
        )
        if not self.realsense.connect():
            self.get_logger().warning("RealSense 相机按配置序列号连接失败，尝试自动匹配设备")
            self.realsense = RealSenseCamera(
                camera_width=REALSENSE_RESOLUTION[0],
                camera_height=REALSENSE_RESOLUTION[1],
                camera_fps=REALSENSE_RESOLUTION[2],
                serial_number=None,
            )
            if not self.realsense.connect():
                self.get_logger().error("RealSense 相机连接失败，录制中将缺失彩色图像")
                self.realsense = None
            else:
                self.get_logger().info("RealSense 相机已通过自动匹配连接")

    def _disconnect_cameras(self) -> None:
        if self.fisheye:
            self.fisheye.disconnect()
        if self.realsense:
            self.realsense.disconnect()

    # ==============================
    # 采集线程
    # ==============================
    def _capture_loop(self) -> None:
        interval = 1.0 / CAPTURE_HZ if CAPTURE_HZ > 0 else 0.1
        while self.running:
            episode_dir = self.episode_dir
            if self.recording and episode_dir:
                self.frame_id += 1
                fid = f"{self.frame_id:05d}"

                # ---- 鱼眼相机 ----
                if self.fisheye:
                    ok, frame = self.fisheye.get_frame()
                    if ok and frame is not None:
                        path = os.path.join(episode_dir, f"img/fisheye_rgb/{fid}.jpg")
                        cv2.imwrite(path, frame)

                # ---- RealSense 相机 ----
                if self.realsense:
                    ok_color, color_frame = self.realsense.get_color_frame()
                    if ok_color and color_frame is not None:
                        color_path = os.path.join(episode_dir, f"img/realsense_rgb/{fid}.jpg")
                        cv2.imwrite(color_path, color_frame)

                # ---- 状态存储 ----
                state_path = os.path.join(episode_dir, f"state/{fid}.log")
                state_data = self.joint_state + [self.gripper_pos]
                with open(state_path, "w") as f:
                    f.write(" ".join(f"{value:.6f}" for value in state_data) + "\n")

            time.sleep(interval)

        self._disconnect_cameras()
        cv2.destroyAllWindows()
        self.get_logger().info("采集线程已安全退出 ✅")


def main() -> None:
    rclpy.init()
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断采集")
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
