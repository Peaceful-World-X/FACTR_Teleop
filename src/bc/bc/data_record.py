# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from pathlib import Path
import pickle
import shutil
import time
from pynput import keyboard
from termcolor import colored

from bc import utils
from python_utils.utils import get_workspace_root


class ObsBuffer:
    """Thread-safe buffer for storing the latest message."""
    def __init__(self):
        self.msg = None
        self.lock = threading.Lock()

    def update(self, msg):
        with self.lock:
            self.msg = msg

    def get(self):
        with self.lock:
            return self.msg


class DataRecord(Node):
    def __init__(self, name="data_record_node"):
        super().__init__(name)

        # 参数声明
        self.declare_parameter('dataset_name', "")
        self.dataset_name = self.get_parameter('dataset_name').value
        
        # 相机话题列表：第一个被视为主触发源
        # 例如：['/realsense/front/im', '/realsense/side/im']
        self.declare_parameter('camera_topics', [])
        self.camera_topics = self.get_parameter('camera_topics').value
        
        # 状态话题列表：包含 Franka, FACTR, Gripper 等的所有 JointState 话题
        self.declare_parameter('state_topics', [])
        self.state_topics = self.get_parameter('state_topics').value

        # 输出目录
        self.output_dir = Path(f"{get_workspace_root()}/raw_data/{self.dataset_name}")
        if not self.output_dir.exists():
            self.output_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Saving to {self.output_dir}")

        # 内部状态
        self.recording = False
        self.current_ep_index = None
        self.current_episode_dir: Path | None = None
        self.current_episode_data = []
        self.current_frame_id = 0
        
        self.bridge = CvBridge()
        
        # 状态缓存：{topic_name: ObsBuffer}
        self.state_buffers = {topic: ObsBuffer() for topic in self.state_topics}
        # 辅助相机缓存：{topic_name: ObsBuffer} (主相机不需要缓存，直接在回调中处理)
        self.camera_buffers = {}
        if len(self.camera_topics) > 1:
            for cam_topic in self.camera_topics[1:]:
                self.camera_buffers[cam_topic] = ObsBuffer()

        # 订阅状态话题
        for topic in self.state_topics:
            if "pose" in topic or "end_effector" in topic:
                msg_type = Float64MultiArray
            else:
                msg_type = JointState

            self.create_subscription(
                msg_type, 
                topic, 
                lambda msg, t=topic: self.state_buffers[t].update(msg), 
                10
            )
            
        # 订阅辅助相机话题
        for topic in self.camera_buffers:
            self.create_subscription(
                Image, 
                topic, 
                lambda msg, t=topic: self.camera_buffers[t].update(msg), 
                1
            )

        # 订阅主相机话题（触发源）
        if self.camera_topics:
            self.main_camera_topic = self.camera_topics[0]
            self.create_subscription(
                Image, 
                self.main_camera_topic, 
                self.main_camera_callback, 
                1
            )
            self.get_logger().info(f"主触发相机: {self.main_camera_topic}")
        else:
            self.get_logger().warn("未配置相机话题，无法触发录制！")

        # 键盘控制
        self.listener = keyboard.Listener(on_press=self.on_press_key)
        self.listener.start()

        self.get_logger().info(colored(f"Ready to record. Press SPACE to start/stop.", 'green'))

    def main_camera_callback(self, msg: Image):
        """主相机回调：触发一帧数据的录制"""
        if not self.recording:
            return

        # 1. 立即获取所有状态快照 (Latest Observation)
        snapshot = {}
        # 状态数据
        for topic, buf in self.state_buffers.items():
            snapshot[topic] = buf.get()
        # 辅助相机图像
        other_cam_imgs = {}
        for topic, buf in self.camera_buffers.items():
            other_cam_imgs[topic] = buf.get()

        # 2. 处理并保存主相机图像
        try:
            main_cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"主相机图像转换失败: {e}")
            return

        frame_id = self.current_frame_id
        
        # 确保图片目录存在
        # 结构：ep_000/camera_name/0000.bmp
        main_cam_name = self.topic_to_name(self.main_camera_topic)
        main_cam_dir = self.current_episode_dir / main_cam_name
        main_cam_dir.mkdir(parents=True, exist_ok=True)
        
        main_img_path = main_cam_dir / f"{frame_id:04d}.bmp"
        cv2.imwrite(str(main_img_path), main_cv_img)

        # 3. 处理并保存辅助相机图像
        saved_img_paths = {main_cam_name: str(main_img_path.relative_to(self.output_dir))}
        
        for topic, img_msg in other_cam_imgs.items():
            if img_msg is None:
                continue
            try:
                cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                cam_name = self.topic_to_name(topic)
                cam_dir = self.current_episode_dir / cam_name
                cam_dir.mkdir(parents=True, exist_ok=True)
                
                img_path = cam_dir / f"{frame_id:04d}.bmp"
                cv2.imwrite(str(img_path), cv_img)
                saved_img_paths[cam_name] = str(img_path.relative_to(self.output_dir))
            except Exception:
                pass

        # 4. 解析并组装状态数据
        frame_data = {
            "frame_id": frame_id,
            "images": saved_img_paths,
        }
        
        # 解析 Franka / FACTR / Gripper 数据
        # 注意：这里需要根据实际话题名进行硬编码解析，或者通用解析
        # 假设话题名如下（需根据您的 launch 文件确认）：
        # Franka State: /franka/joint_states
        # Gripper: /bridge/obs_gripper_state
        # FACTR: /factr/joint_states (假设)
        
        for topic, state_msg in snapshot.items():

            if state_msg is None:
                continue
            
            # 1. 处理位姿数据 (Float64MultiArray)
            if isinstance(state_msg, Float64MultiArray):
                data = np.array(state_msg.data, dtype=np.float64)
                # 如果是 16 维数据，假定为 4x4 齐次矩阵（列主序）
                if data.size == 16:
                    frame_data[topic] = data.reshape((4, 4), order='F')
                else:
                    frame_data[topic] = data
                continue

            # 通用解析：直接存 position/velocity/effort
            # 若需要特定字段重命名，可在此处添加逻辑
            # 例如：
            # if "franka" in topic: ...

            
            # 特殊处理：如果是夹爪状态，提取开度
            if "/bridge/obs_gripper_state" in topic:
                # position[0] = leader_ratio, position[1] = follower_ratio
                if hasattr(state_msg, 'position') and len(state_msg.position) >= 2:
                    frame_data["leader_gripper"] = state_msg.position[0]
                    frame_data["franka_gripper"] = state_msg.position[1]
                continue

            # 针对 specific topics 的解析
            if isinstance(state_msg, JointState):
                processed_msg = {}
                if state_msg.position:
                    processed_msg['position'] = np.array(state_msg.position, dtype=np.float64)
                if state_msg.velocity:
                    processed_msg['velocity'] = np.array(state_msg.velocity, dtype=np.float64)
                if state_msg.effort:
                    processed_msg['effort'] = np.array(state_msg.effort, dtype=np.float64)
            else:
                processed_msg = utils.process_msg(state_msg)
            
            # 存入 frame_data，键名为 topic
            frame_data[topic] = processed_msg

        self.current_episode_data.append(frame_data)
        self.current_frame_id += 1

    def topic_to_name(self, topic):
        """将话题名转换为目录名（去除斜杠）"""
        return topic.strip('/').replace('/', '_')

    def start_recording(self):
        # 确定新的 episode index
        all_episodes = [d for d in self.output_dir.iterdir() if d.is_dir() and d.name.startswith('ep_')]
        self.current_ep_index = len(all_episodes)
        
        episode_name = f"ep_{self.current_ep_index:05d}"
        self.current_episode_dir = self.output_dir / episode_name
        self.current_episode_dir.mkdir(parents=True, exist_ok=True)
        
        self.current_episode_data = []
        self.current_frame_id = 0
        self.recording = True
        self.get_logger().info(f"Started recording: {episode_name}")

    def stop_recording(self):
        self.recording = False
        if not self.current_episode_data:
            self.get_logger().warn("No data recorded!")
            # 清理空目录
            if self.current_episode_dir and self.current_episode_dir.exists():
                shutil.rmtree(self.current_episode_dir)
            return

        # 保存 PKL
        pkl_path = self.current_episode_dir / "data.pkl"
        with open(pkl_path, 'wb') as f:
            pickle.dump(self.current_episode_data, f, protocol=pickle.HIGHEST_PROTOCOL)
            
        self.get_logger().info(f"Saved {len(self.current_episode_data)} frames to {pkl_path}")
        self.current_episode_dir = None
        self.current_episode_data = []

    def delete_last_episode(self):
        all_episodes = sorted([d for d in self.output_dir.iterdir() if d.is_dir() and d.name.startswith('ep_')])
        if not all_episodes:
            self.get_logger().info("No episodes to delete.")
            return
            
        last_ep = all_episodes[-1]
        shutil.rmtree(last_ep)
        self.get_logger().info(colored(f"Deleted {last_ep.name}", 'red'))

    def on_press_key(self, key):
        try:
            if key == keyboard.Key.space:
                if not self.recording:
                    self.start_recording()
                else:
                    self.stop_recording()
            elif key == keyboard.Key.delete:
                if not self.recording:
                    self.delete_last_episode()
            elif hasattr(key, 'char') and key.char == 'q':
                # 添加 'q' 键退出功能
                self.get_logger().info("收到退出信号，正在停止...")
                if self.recording:
                    self.stop_recording()
                self.listener.stop()
                rclpy.shutdown()
        except AttributeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DataRecord()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
