import tensorflow_datasets as tfds
import numpy as np
import cv2
import glob
import os

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
from pathlib import Path


_DESCRIPTION = "Piper Robot Dataset from ROS2 Bags"


class PiperDatasetBuilder(tfds.core.GeneratorBasedBuilder):
    VERSION = tfds.core.Version('1.0.0')

    def _info(self):
        return tfds.core.DatasetInfo(
            builder=self,
            description=_DESCRIPTION,
            features=tfds.features.FeaturesDict({
                'steps': tfds.features.Dataset({
                    'observation': tfds.features.FeaturesDict({
                        'image': tfds.features.Image(shape=(480, 640, 3), dtype=np.uint8),
                        'state': tfds.features.Tensor(shape=(7,), dtype=np.float32),
                    }),
                    'action': tfds.features.Tensor(shape=(7,), dtype=np.float32),
                    'language_instruction': tfds.features.Text(),
                    'is_first': tfds.features.Scalar(dtype=np.bool_),
                    'is_last': tfds.features.Scalar(dtype=np.bool_),
                    'is_terminal': tfds.features.Scalar(dtype=np.bool_),
                }),
                'episode_metadata': tfds.features.FeaturesDict({
                    'file_path': tfds.features.Text(),
                }),
            }),
        )

    def _split_generators(self, dl_manager):
        return {
            'train': self._generate_examples(
                path="/home/agv/wzb/python_code/piper_dataset_builder/rosbag"
            ),
        }

    def _generate_examples(self, path):
        bag_folders = glob.glob(os.path.join(path, "my_task_dataset_*"))

        for bag_path in bag_folders:
            episode_data = self._process_bag(bag_path)
            if len(episode_data) > 0:
                yield bag_path, {
                    'steps': episode_data,
                    'episode_metadata': {'file_path': bag_path}
                }

    def _process_bag(self, bag_path):
        images = []
        joints = []

        typestore = get_typestore(Stores.ROS2_HUMBLE)  # ⭐ 根据你系统 ROS2 版本改

        with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
            connections = [c for c in reader.connections]

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)

                # ---------- IMAGE ----------
                if connection.topic == '/camera/color/image_raw/compressed':
                    if msg.encoding.lower() in ["rgb8", "bgr8"]:
                        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                        if msg.encoding.lower() == "bgr8":
                            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

                    elif msg.encoding.lower() == "mono8":
                        mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                        img = cv2.cvtColor(mono, cv2.COLOR_GRAY2RGB)
                    else:
                        print("Unsupported encoding:", msg.encoding)
                        continue

                    img_resized = cv2.resize(img, (640, 480))

                    if hasattr(msg, "header") and msg.header.stamp.sec != 0:
                        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    else:
                        t = timestamp * 1e-9

                    images.append({'time': t, 'data': img_resized})

                # ---------- JOINT STATE ----------
                elif connection.topic == '/joint_states_feedback':
                    names = list(msg.name)
                    positions = list(msg.position)

                    joint_map = dict(zip(names, positions))

                    joint_angles = [joint_map.get(f'joint{i+1}', 0.0) for i in range(6)]
                    gripper_val = joint_map.get('gripper', 0.0)

                    ee_pose = self.compute_piper_fk(joint_angles)
                    gripper_norm = 1.0 if gripper_val > 0.02 else 0.0

                    state_vector = np.array(ee_pose + [gripper_norm], dtype=np.float32)

                    if hasattr(msg, "header") and msg.header.stamp.sec != 0:
                        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    else:
                        t = timestamp * 1e-9

                    joints.append({'time': t, 'data': state_vector})

        # ---------- 时间对齐 ----------
        aligned_steps = []
        for img_entry in images:
            img_time = img_entry['time']
            closest_joint = min(joints, key=lambda x: abs(x['time'] - img_time))

            if abs(closest_joint['time'] - img_time) > 0.1:
                continue

            aligned_steps.append({
                'image': img_entry['data'],
                'state': closest_joint['data']
            })

        # ---------- 构建 Episode ----------
        final_episode = []
        task_instruction = "pick up the object"

        for i in range(len(aligned_steps) - 1):
            current_state = aligned_steps[i]['state']
            next_state = aligned_steps[i + 1]['state']

            delta_pose = (next_state[:6] - current_state[:6]).astype(np.float32)
            gripper_action = np.float32(next_state[6])

            action = np.concatenate([delta_pose, [gripper_action]]).astype(np.float32)

            final_episode.append({
                'observation': {
                    'image': aligned_steps[i]['image'],
                    'state': current_state.astype(np.float32),
                },
                'action': action,
                'is_first': i == 0,
                'is_last': i == (len(aligned_steps) - 2),
                'is_terminal': i == (len(aligned_steps) - 2),
                'language_instruction': task_instruction
            })

        return final_episode

    # ---------------- FK ----------------
    def compute_piper_fk(self, joint_angles):
        dh_params = [
            [-np.pi/2,  0.0,        0.123,    0.0],
            [0.0,       0.28503,    0.0,      -172.22 * np.pi / 180.0],
            [np.pi/2,   -0.021984,  0.0,      -102.78 * np.pi / 180.0],
            [-np.pi/2,  0.0,        0.25075,  0.0],
            [np.pi/2,   0.0,        0.0,      0.0],
            [0.0,       0.0,        0.211,    0.0]
        ]

        T_total = np.eye(4)

        for i in range(6):
            theta = joint_angles[i]
            alpha, a, d, theta_offset = dh_params[i]
            theta_final = theta + theta_offset

            ct, st = np.cos(theta_final), np.sin(theta_final)
            ca, sa = np.cos(alpha), np.sin(alpha)

            T_i = np.array([
                [ct, -st*ca,  st*sa, a*ct],
                [st,  ct*ca, -ct*sa, a*st],
                [0,      sa,     ca,    d],
                [0,       0,      0,    1]
            ])

            T_total = T_total @ T_i

        x, y, z = T_total[0, 3], T_total[1, 3], T_total[2, 3]
        sy = np.sqrt(T_total[0, 0]**2 + T_total[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(T_total[2, 1], T_total[2, 2])
            pitch = np.arctan2(-T_total[2, 0], sy)
            yaw = np.arctan2(T_total[1, 0], T_total[0, 0])
        else:
            roll = np.arctan2(-T_total[1, 2], T_total[1, 1])
            pitch = np.arctan2(-T_total[2, 0], sy)
            yaw = 0.0

        return [x, y, z, roll, pitch, yaw]


if __name__ == "__main__":
    builder = PiperDatasetBuilder()
    builder.download_and_prepare()
    print("✅ 数据集转换完成！")