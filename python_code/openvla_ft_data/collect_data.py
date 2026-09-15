import time
import cv2
import pickle
import numpy as np
from datetime import datetime
import os

# 假设你已经配置好了 Piper SDK
# from piper_sdk import PiperRobot 

def collect_episode(robot, camera_id=0, save_dir="/home/agv/wzb/python_code/openvla_ft_data/raw_data"):
    # 1. 准备环境
    cap = cv2.VideoCapture(camera_id)
    # 设置分辨率，OpenVLA 默认训练分辨率通常是 224x224，但建议录制时存 640x480 以保留细节
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    episode_data = []
    task_description = input("请输入当前任务指令 (例如: put the apple in the bowl): ")
    
    print("3秒后开始录制...请准备拖动机械臂")
    time.sleep(3)
    print("开始录制！按 'q' 结束当前条目。")

    # 开启 Piper 的零力/拖拽模式
    # robot.set_mode("passive") 或 robot.enable_drag_teaching()
    
    try:
        while True:
            timestamp = time.time()
            
            # A. 获取图像
            ret, frame = cap.read()
            if not ret: break
            
            # B. 获取机械臂状态 (末端位姿: x, y, z, roll, pitch, yaw, gripper_width)
            # 这一步非常关键，必须记录末端执行器 (End Effector) 的数据
            # piper_state = robot.get_end_effector_pose() 
            # gripper_state = robot.get_gripper_state()
            
            # 模拟数据 (替换为你的真实 SDK 调用)
            current_ee_pose = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0]) 
            gripper_open = 1.0 # 1.0 open, 0.0 closed
            
            # C. 组装一帧数据
            step_data = {
                "image": frame, # 保存原始 RGB 数据
                "state": np.concatenate([current_ee_pose, [gripper_open]]), # 7维向量
                "language": task_description,
                "timestamp": timestamp
            }
            episode_data.append(step_data)
            
            # 显示画面
            cv2.imshow('Recording', frame)
            if cv2.waitKey(50) & 0xFF == ord('q'): # 20Hz 左右采样
                break
                
    finally:
        cap.release()
        cv2.destroyAllWindows()
        # robot.set_mode("active") # 恢复控制模式

    # 3. 保存数据
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = os.path.join(save_dir, f"episode_{timestamp_str}.pkl")
    with open(save_path, 'wb') as f:
        pickle.dump(episode_data, f)
    print(f"数据已保存至: {save_path}, 共 {len(episode_data)} 帧")

if __name__ == "__main__":
    collect_episode(None)