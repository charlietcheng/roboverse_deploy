from src.client import HttpPolicyClient
import transforms3d as t3d
import numpy as np

class BaseAgent:
    def step(self, rgb, history, instruction, curent_pose):
        raise NotImplementedError
    

def get_eef_pose_from_hand_pose(p, q):
    return p + t3d.quaternions.rotate_vector([0,0,0.1034], q), q    

def fk(joint_angles):
    dh_params = [[0, 0.333, 0, joint_angles[0]],
        [0, 0, -np.pi/2, joint_angles[1]],
        [0, 0.316, np.pi/2, joint_angles[2]],
        [0.0825, 0, np.pi/2, joint_angles[3]],
        [-0.0825, 0.384, -np.pi/2, joint_angles[4]],
        [0, 0, np.pi/2, joint_angles[5]],
        [0.088, 0, np.pi/2, joint_angles[6]],
        [0, 0.107, 0, 0],
        [0, 0, 0, -np.pi/4],
        [0.0, 0.000, 0, 0]]


    def get_tf_mat(i, dh):
        a = dh[i][0]
        d = dh[i][1]
        alpha = dh[i][2]
        theta = dh[i][3]
        q = theta
        return np.array([[np.cos(q), -np.sin(q), 0, a],
                        [np.sin(q) * np.cos(alpha), np.cos(q) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
                        [np.sin(q) * np.sin(alpha), np.cos(q) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
                        [0, 0, 0, 1]])
    
    T = np.eye(4)
    for i in range(10):
        T = T @ get_tf_mat(i, dh_params)

    p = T[:3, 3]
    # q = t3d.quaternions.mat2quat(T[:3, :3])
    # return p, q
    R = T[:3, :3]
    return p, R

class AgentDP: 
    def __init__(self):
        # use this ip when starts with docker compose
        self.client = HttpPolicyClient(server_ip="127.0.0.1", server_port=26785)

    def step(self, rgb_frame, history, instruction, eef_pose):
        print("history: ", history)
        history = [np.array(history[-1]).reshape(-1, 9)]
        rgb_frame = np.array(rgb_frame)
        # Crop to 256x256 from the center of the image
        h, w = rgb_frame.shape[:2]
        start_h = (h - 256) // 2
        start_w = (w - 256) // 2
        rgb_frame = rgb_frame[start_h:start_h+256, start_w:start_w+256, :]
        actions = self.client.query_action(rgb_frame, instruction, history = history)
        eef_actions = []
        for act in actions[0]:
            p_eef, mat_eef = fk(act[:8])
            gripper = 1 if act[8] > 0.39 else 0  # 1: open,  0: close
            eef_actions.append((p_eef, mat_eef, gripper))
        return eef_actions        

    def query_action(self, frame, instruction):
        action = self.client.query_action(frame, instruction)
        return action # delta action in eef space

class AgentOpenvla: 
    def __init__(self):
        # use this ip when starts with docker compose
        self.client = HttpPolicyClient(server_ip="127.0.0.1", server_port=26784)

    def step(self, rgb_frame, history, instruction, eef_pose):
        # frame = np.zeros((360, 640, 3), dtype=np.uint8)
        action = self.client.query_action(rgb_frame, instruction)
        print("action: ", action)

        delta_actions = [action]
        current_position = eef_pose[:3]
        # current_orientation_mat = t3d.quaternions.quat2mat(eef_pose[3:7])
        abs_actions = []
        cnt = 0
        for action in delta_actions:
            assert action[6] in [1., 0.], "please quantize gripper actions before passing it to me"
            target_position = current_position + action[:3]
            delta_ori = t3d.euler.euler2quat(*action[3:6])
            q1 = t3d.quaternions.qmult(delta_ori, eef_pose[3:7])
            target_orientation_mat = t3d.quaternions.quat2mat(q1)
            
            abs_actions.append((target_position, target_orientation_mat, action[6]))
            # self.robot.update_tf('target_' + str(cnt), abs_actions[cnt][0], t3d.euler.mat2euler(abs_actions[cnt][1]))
            cnt = cnt + 1
            current_position = target_position
            # current_orientation_mat = target_orientation_mat
            # self.idx += 1
        # print(f"action: {[abs_actions[0]]}")
        return abs_actions, None


    def query_action(self, frame, instruction):
        action = self.client.query_action(frame, instruction)
        return action # delta action in eef space

class AgentOcto:
    def __init__(self):
        # use this ip when starts with docker compose
        self.client = HttpPolicyClient(server_ip="127.0.0.1")
        self.actions_buffer  = deque(maxlen=10)

    def query_action(self, frame, instruction):
        if len(self.actions) == 0:
            actions = self.client.query_action(frame, instruction)
            for act in actions:
                self.actions_buffer.add(act)
