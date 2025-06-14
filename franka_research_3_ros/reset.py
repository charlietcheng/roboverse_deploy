import numpy as np
import rospy
from src.robot import FrankaPanda
from src.camera import CameraRealsense
from src.agent import AgentDP, AgentOpenvla, fk, get_eef_pose_from_hand_pose
from src.visualize import RvizVisualizer
from src.video_recorder import VideoRecoder
from src.task import GraspTask
import transforms3d as t3d

# q1 = [2.002886984509563e-05, -1.300138203817897, -6.955714721409962e-05, -2.4999925341020552, -0.00027023131110601954, 1.000085030009851, 6.952861831483755e-05]
# q2 = [0, -1.3, 0, -2.5, 0, 1, 0]
# result = np.allclose(q1, q2, atol=1e-2)

# print(result)

print("IN RESET")

rospy.init_node('franka_control', anonymous=True, disable_signals=True)

print("node inited")

viz = RvizVisualizer()
video_recorder = VideoRecoder("test.mp4")
camera = CameraRealsense() # None #
# import pdb; pdb.set_trace()
robot = FrankaPanda() # None # 
print("robot loaded")

target_qpos = [
        0.09162008114028396,
        -0.19826458111314524,
        -0.01990020486871322,
        -2.4732269941140346,
        -0.01307073642274261,
        2.30396583422025,
        0.8480939705504309,
    ]
p, R = fk(target_qpos)
# print(hand_pos)
# eef_pos = (hand_pos[0], hand_pos[1], np.array([1.0]))
q = t3d.quaternions.mat2quat(R)


p1, q1  = get_eef_pose_from_hand_pose(p, q)

print("====reset pose===")
print(p1)
print(q1)
print("=================")
R1 = t3d.quaternions.quat2mat(q1)
# robot.move_to([p1, R1, 1.0])
robot.execute_waypoints([[p1, R1, 1.0]], timeout=5.0)
