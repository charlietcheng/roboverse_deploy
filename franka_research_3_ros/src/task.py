import os
import uuid
import numpy as np
from PIL import Image
import transforms3d as t3d
from src.robot import FrankaPanda
from src.camera import CameraRealsense
from src.agent import AgentDP, AgentOpenvla, BaseAgent
import pickle

class GraspTask:
    def __init__(self, agent, camera, robot):
        self.agent:BaseAgent = agent
        self.camera:CameraRealsense = camera
        self.robot:FrankaPanda = robot
        self.collect_action = False
        self.replay_traj = False

        if self.replay_traj:
            # with open(f"exp/e068c62e-19d5-4c8e-ac8a-4a27346f3623/history.pkl", "rb") as f:
            #     self.actions = pickle.load(f)

            self.actions = []
            with open(f"dump_05_p000.txt", "r") as f:
                lines = [line.rstrip() for line in f]
                for line in lines:
                    # print(str(line))
                    p = np.array([float(x) for x in line.split(" ")[:3]])
                    q = np.array([float(x) for x in line.split(" ")[3:7]])
                    gripper = float(line.split(" ")[7])
                    self.actions.append([[(p, t3d.quaternions.quat2mat(q), gripper)], None])
        
        # Generate a random UUID (version 4)
        self.uuid_str = str(uuid.uuid4())
        os.makedirs(f"exp/{self.uuid_str}", exist_ok=True)
        
        self.idx = 0 
        self.history = []

    def run(self, instruction = None):
        # print("debug: ", self.idx)
        if self.idx == 0: # reset gripper
            self.robot.open_gripper()

        if self.idx == 0:
            if type(self.agent) is AgentDP:
                proprio_qpos = self.robot.latest_qpos
                proprio_gripper = 1 if self.robot.gripper_status  == "open" else 0
                # print(f"proprio qpos: {proprio_qpos}")
                # print(f"proprio gripper: {proprio_gripper}")
                # print("L51")
                # print(proprio_qpos.shape)
                # print(proprio_gripper)
                proprio = np.concatenate([proprio_qpos, [proprio_gripper, proprio_gripper]])
                print(f"proprio: {proprio}")
                self.history.append(proprio)
            elif type(self.agent) is AgentOpenvla:
                # not tested
                # proprio = self.robot.get_eef_pose()
                # print(f"proprio eef: {proprio}")
                # self.history.append(proprio)
                pass
            else:
                raise ValueError("history not set correctly")
        
        # if eef closed to 0, force open and re-try
        # if self.robot.get_eef_width() < 0.0013:
        #     print("force open gripper")
        #     self.robot.open_gripper()
        #     recover_action = self.last_eef
        #     # print(f"last action: {recover_action}")
        #     # recover_action[0][2] += 0.03
        #     # print(f"recover action: {recover_action}")
        #     self.robot.execute_waypoints([recover_action], timeout=1.0)
        #     # self.history = [recover_action] # reset history with only one-step action

        rgb_frame, _ = self.camera.capture_frame()

        # if type(self.agent) is AgentDP:
        #     h, w, _ = rgb_frame.shape
        #     top = (h - 256) // 2
        #     left = (w - 256) // 2
        #     rgb_frame = rgb_frame[top:top + 256, left:left + 256]
        
        # save images
        Image.fromarray(rgb_frame).save(f"exp/{self.uuid_str}/req_{self.idx}.png")

        # prev_eef_pose = self.robot.get_eef_pose(0.3)
        eef_pose = self.robot.get_eef_pose()
        # print(f"prev eef pose: {prev_eef_pose}")
        # print(f"curr eef pose: {eef_pose}")

        abs_actions, raw_actions = self.agent.step(rgb_frame, self.history, instruction, eef_pose) if not self.replay_traj else self.actions[self.idx]
        print("abs_actions: ", abs_actions)
        self.last_eef = abs_actions[-1]
        self.robot.execute_waypoints(abs_actions, timeout=5.0)
        if raw_actions is not None:
            print("position error: ", np.linalg.norm(self.robot.get_dof_pose() - raw_actions[-1]))
        dof_pose = self.robot.get_dof_pose()
        self.history.append(dof_pose)
        # self.history.append(raw_actions[0][:8])
        # self.history.append(raw_actions[1][:8])
        self.idx += 2

        if self.collect_action:
            with open(f"exp/{self.uuid_str}/history.pkl", "ab") as f:
                pickle.dump({self.idx: abs_actions}, f)
            