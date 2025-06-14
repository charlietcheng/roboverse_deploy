import time
import rospy
import tf2_ros
import actionlib
from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped
from sensor_msgs.msg import JointState
from franka_gripper.msg import GraspGoal, GraspAction, MoveAction, MoveGoal
from franka_msgs.msg import FrankaState, ErrorRecoveryAction, ErrorRecoveryGoal
from dynamic_reconfigure.client import Client

import numpy as np
import threading
from collections import deque
import transforms3d as t3d
import time

MIN_HEIGHT = 0.048
MAX_HEIGHT = 0.6
POS_TOL = 0.005
ROT_TOL = 3/180 * np.pi
FORCE_LIMIT = 10
STICKY_GRIPPER_COUNT = 0

class FrankaPanda:
    
    REAL_EEF_TO_SIM_EEF = np.array([
        [1., 0., 0., 0.],
        [0., 1., 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ])

    ROBOT_MODE_OTHER=0
    ROBOT_MODE_IDLE=1
    ROBOT_MODE_MOVE=2
    ROBOT_MODE_GUIDING=3
    ROBOT_MODE_REFLEX=4
    ROBOT_MODE_USER_STOPPED=5
    ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
    ROBOT_MODES_UNCONTROLLABLE = [ROBOT_MODE_GUIDING, ROBOT_MODE_USER_STOPPED, ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY]

    BASE_FRAME_ID = 'panda_link0'
    EEF_FRAME_ID = 'panda_EE'

    def __init__(self):
        self.robot_mode = None
        self.robot_mode_ts = 0.
        self.gripper_status = None # default gripper state
        self.abnormal = False
        self._timestamp = time.monotonic()
        
        self.eef_pose_publisher = rospy.Publisher('/cartesian_impedance_controller/equilibrium_pose', PoseStamped, queue_size=0)
        
        self.latest_qpos = None
        self.franka_state_subscriber = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self._franka_state_callback)
        self.franka_gripper_state_subscriber = rospy.Subscriber('/franka_gripper/joint_states', JointState, self._franka_gripper_state_callback)

        #self.franka_f_ext_subscriber = rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self._franka_f_ext_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # self.gripper_grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        # self.gripper_grasp_client.wait_for_server()
        # self.gripper_move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        # self.gripper_move_client.wait_for_server()
        self.error_recovery_client = actionlib.SimpleActionClient('/franka_control/error_recovery', ErrorRecoveryAction)
        self.error_recovery_client.wait_for_server()

        self.reset_history()
        self.robot_lock = threading.RLock()

        self.dynamic_param_client = Client("/cartesian_impedance_controllerdynamic_reconfigure_compliance_param_node")
        # self.dynamic_param_client.update_configuration({"filter_d_order": 3})
        self.dynamic_param_client.update_configuration({"filter_params": 0.004})
        self.dynamic_param_client.update_configuration({"translational_stiffness": 2000.0})
        self.dynamic_param_client.update_configuration({"translational_damping": 89.0})
        self.dynamic_param_client.update_configuration({"rotational_stiffness": 150.0})
        self.dynamic_param_client.update_configuration({"rotational_damping": 7.0})
        self.dynamic_param_client.update_configuration({"nullspace_stiffness": 0.2})
        self.dynamic_param_client.update_configuration({"joint1_nullspace_stiffness": 100.0})
        for direction in ['x', 'y', 'z', 'neg_x', 'neg_y', 'neg_z']:
            self.dynamic_param_client.update_configuration({"translational_clip_" + direction: 0.02})
            self.dynamic_param_client.update_configuration({"rotational_clip_" + direction: 0.05})  
        self._check_initial_state()
        # self._try_recover_from_error() # TODO what is this for?

        
        
        
    def step(self, action, controller = "eef"):
        if controller == "eef":
            print("received action: ", action)
            eef_pose = action
        elif controller == "joint":
            # eef_pose = self.get_eef_pose_from_joint()
            eef_pose = action
        else:
            raise ValueError("invalid controller type")
        
        self.move_to(eef_pose)

    def get_eef_pose(self, before=0.):
        if len(self.history_waypoints) == 0:
            self.history_waypoints.append(self._get_eef_pose_from_ros())
        idx = round(before / 0.1)
        if idx + 1 > len(self.history_waypoints):
            waypoint = self.history_waypoints[0]
        else:
            waypoint = self.history_waypoints[-(1+idx)]
        # return np.array([*waypoint[0], *t3d.euler.mat2euler(waypoint[1]), waypoint[2]])
        return np.array([*waypoint[0], *t3d.quaternions.mat2quat(waypoint[1]), waypoint[2]])
    
    def get_dof_pose(self):
        gripper_status = [1, 1] if self.gripper_status == 'open' else [0, 0]
        return np.concatenate((self.latest_qpos, gripper_status))
    
    def get_eef_width(self):
        return sum(self.gripper_joint_positions)

    def update_tf(self, frame_id, xyz, rpy):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.BASE_FRAME_ID
        msg.child_frame_id = frame_id
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
        msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z = t3d.euler.euler2quat(*rpy)
        self.tf_broadcaster.sendTransform(msg)

    def _get_eef_pose_from_ros(self):
        trans = self.tf_buffer.lookup_transform(self.BASE_FRAME_ID, self.EEF_FRAME_ID, rospy.Time.now(), rospy.Duration(0.5))
        real_pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        real_ori_quat_sxyz = np.array([trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z])

        real_pose_mat = np.eye(4)
        real_pose_mat[:3, 3] = real_pos
        real_pose_mat[:3, :3] = t3d.quaternions.quat2mat(real_ori_quat_sxyz)
        sim_pose_mat = real_pose_mat @ self.REAL_EEF_TO_SIM_EEF
        sim_pos = sim_pose_mat[:3, 3]
        return (sim_pos, sim_pose_mat[:3, :3], 1 if self.gripper_status == 'open' else -1)

    def get_current_eef_pose(self, before):
        while True:
            try:
                trans = self.tf_buffer.lookup_transform(self.BASE_FRAME_ID, self.EEF_FRAME_ID, rospy.Time.now() - rospy.Duration(before), rospy.Duration(0.5))
                break
            except tf2_ros.ExtrapolationException as e:
                continue

        real_pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        real_ori_quat_sxyz = np.array([trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z])
        real_pose_mat = np.eye(4)
        real_pose_mat[:3, 3] = real_pos
        real_pose_mat[:3, :3] = t3d.quaternions.quat2mat(real_ori_quat_sxyz)
        sim_pose_mat = real_pose_mat @ self.REAL_EEF_TO_SIM_EEF
        sim_pos = sim_pose_mat[:3, 3]
        return (sim_pos, sim_pose_mat[:3, :3], 1 if self.gripper_status == 'open' else -1)

    def move_to(self, position):
        self.execute_waypoints_and_wait([position])

    def open_gripper(self):
        self.move_gripper('open')

    def close_gripper(self):
        self.move_gripper('close')

    def execute_waypoints_and_wait(self, waypoints, timeout=2.0):
        self.execute_waypoints(waypoints ,timeout)
        # if timeout == 0:
        #     return
        try:
            self.wait_for_reach(0.005, 5/180*np.pi, timeout=timeout)
        except TimeoutError as e:
            print(f"controller timeout: {e}")

    def reset_history(self):
        self.history_waypoints = deque(maxlen=100)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # wait for the first transform
        time.sleep(0.3)

    def wait_for_reach(self, pos_tol, rot_tol, timeout):
        self._wait_for_reach(*self.last_waypoint, pos_tol, rot_tol, timeout)

    def _wait_for_reach(self, pos, rot_mat, pos_tol, rot_tol, timeout, ignore_error=False):
        begin_time = time.time()
        while True:
            if self.abnormal and not ignore_error:
                raise RuntimeError('robot abnormal')
            latest_mat = np.array(self.latest_franka_state.O_T_EE).reshape((4,4)).T @ self.REAL_EEF_TO_SIM_EEF
            pos2 = latest_mat[:3, 3]
            rot_mat2 = latest_mat[:3, :3]
            pos_diff = np.linalg.norm(pos - pos2)
            rot_diff = abs(np.arccos((np.trace(rot_mat @ rot_mat2.T) - 1) / 2))
            if pos_diff < pos_tol and rot_diff < rot_tol:
                break
            spent_time = time.time() - begin_time
            if spent_time > timeout:
                raise TimeoutError(f"control timeout with pos_diff {pos_diff:.4f}m angle_diff {rot_diff / np.pi * 180:.2f}d spent_time {spent_time:.4f}s")
            time.sleep(0.1)

    def execute_waypoints(self, waypoints, timeout, blocking=True):
        """
        This function assumes the given waypoints are near each other.
        If not, please use multiple calls to this function.
        """

        assert blocking, "non-blocking mode is not supported"
        with self.robot_lock:
            waypoints = [((*pos[:2], min(max(pos[2], MIN_HEIGHT), MAX_HEIGHT)), ori_mat, gripper_action) for pos, ori_mat, gripper_action in waypoints]
            for waypoint in waypoints:
                """ if self.robot_mode in self.UNCONTROLLABLE_MODES:
                    print(f'robot in uncontrollable mode, waiting...')
                    while self.robot_mode in self.UNCONTROLLABLE_MODES:
                        time.sleep(0.1)
                    self._recover_from_error()
                    self.history_waypoints.clear()
                    print(f'robot back to normal mode')

                if self.robot_mode in self.ABNORMAL_MODES:
                    print(f'robot in abnormal mode, trying to recover...')
                    self._recover_from_error()
                    print(f'robot back to normal mode') """

                pos, ori_mat, gripper_action = waypoint
                mat = np.eye(4)
                mat[:3, 3] = pos
                mat[:3, :3] = ori_mat
                mat = mat @ np.linalg.inv(self.REAL_EEF_TO_SIM_EEF)

                msg = PoseStamped()
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = mat[:3, 3]
                msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = t3d.quaternions.mat2quat(mat[:3, :3])
                try:
                    self.eef_pose_publisher.publish(msg)
                except Exception as e:
                    print("L234", e)
                # intermediate waypoints can have larger tolerance
                try:
                    self._wait_for_reach(waypoint[0], waypoint[1], pos_tol=0.01, rot_tol=10/180*np.pi, timeout=0.1)
                except TimeoutError as e:
                    print("L239", e)

                # now gripper
                desired_gripper_status = None
                if gripper_action == 0. and self.gripper_status != 'close':
                    desired_gripper_status = 'close'
                elif gripper_action == 1. and self.gripper_status != 'open':
                    desired_gripper_status = 'open'
                if desired_gripper_status is not None:
                    try:
                        self._wait_for_reach(waypoint[0], waypoint[1], pos_tol=POS_TOL, rot_tol=3/180*np.pi, timeout=timeout)
                    except TimeoutError as e:
                        print(f"L251 control: {e}")
                    self.move_gripper(desired_gripper_status)

            try:
                self._wait_for_reach(waypoints[-1][0], waypoints[-1][1], pos_tol=POS_TOL, rot_tol=3/180*np.pi, timeout=timeout)
                self.history_waypoints.extend(waypoints)
            except TimeoutError as e:
                print(f"L258 control: {e}")
                self.history_waypoints.clear()
    
    def _execute_waypoints_physical(self, waypoints):
        has_grasp = False
        for pos, ori_mat, gripper_action in waypoints:
            desired_gripper_status = None
            if gripper_action == -1. and self.gripper_status != 'close':
                desired_gripper_status = 'close'
            elif gripper_action == 1. and self.gripper_status != 'open':
                desired_gripper_status = 'open'
            if desired_gripper_status is not None:
                has_grasp = desired_gripper_status == 'close'
                time.sleep(0.4)
                if self.abnormal:
                    raise RuntimeError('robot abnormal')
                self.move_gripper(desired_gripper_status)
                break

        if not has_grasp:
            if self.abnormal:
                raise RuntimeError('robot abnormal')
            self.set_waypoint(*waypoints[-1][:2])

    def set_waypoint(self, pos, ori_mat):
        mat = np.eye(4)
        mat[:3, 3] = pos
        mat[:3, :3] = ori_mat
        mat = mat @ np.linalg.inv(self.REAL_EEF_TO_SIM_EEF)

        msg = PoseStamped()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = mat[:3, 3]
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = t3d.quaternions.mat2quat(mat[:3, :3])
        self.eef_pose_publisher.publish(msg)
        self.last_waypoint = (pos, ori_mat)

    def move_gripper(self, status):
        with self.robot_lock:
            if status == 'open':
                goal = MoveGoal(width=0.08, speed=0.1)
                self.gripper_move_client.send_goal(goal)
                self.gripper_move_client.wait_for_result(timeout=rospy.Duration(1))
            elif status == 'close':
                goal = GraspGoal(width=0.04, speed=0.2, force=30)
                goal.epsilon.inner = 0.04
                goal.epsilon.outer = 0.04
                self.gripper_grasp_client.send_goal(goal)
                self.gripper_grasp_client.wait_for_result(timeout=rospy.Duration(1))
            self.gripper_status = status

    def _recover_from_error(self):
        with self.robot_lock:
            goal = ErrorRecoveryGoal()
            self.error_recovery_client.send_goal(goal)
            # the error recovery may not response if the robot is not in error mode
            # and there is no reliable way to check if the robot is in error mode
            # so we just wait for a fixed time
            self.error_recovery_client.wait_for_result(rospy.Duration(2))

    def _franka_state_callback(self, msg):
        # print(f"franka state: {msg}")
        self.latest_qpos = np.array(msg.q)

        """ time_elaped = time.monotonic() - self._timestamp
        if time_elaped > 3:
            self._timestamp = time.monotonic()
            print("latest franka qpos", self.latest_qpos) """
        
        self.latest_franka_state = msg
        if self.robot_mode in self.ROBOT_MODES_UNCONTROLLABLE and msg.robot_mode not in self.ROBOT_MODES_UNCONTROLLABLE:
            self.reset_history()
        self.robot_mode = msg.robot_mode
        self.robot_mode_ts = time.time()

    def _franka_gripper_state_callback(self, msg):
        # print("gripper joint states: ", msg)
        self.gripper_joint_positions = msg.position
        self.gripper_status = 'open' if msg.position[0] > 0.039 else 'close'
        """ time_elaped = time.monotonic() - self._timestamp
        if time_elaped > 3:
            self._timestamp = time.monotonic()
            print("GRIPPER status", self.gripper_status) """

    def _franka_f_ext_callback(self, msg):
        # print("external force: ", msg)
        self.external_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    def _try_recover_from_error(self):
        rate = rospy.Rate(50)
        now = time.time()
        while True:
            rate.sleep()
            if self.robot_mode_ts < now:
                continue
            if self.robot_mode not in self.ROBOT_MODES_UNCONTROLLABLE and self.robot_mode != self.ROBOT_MODE_MOVE:
                break
            if self.robot_mode == self.ROBOT_MODE_MOVE:
                return
        goal = ErrorRecoveryGoal()
        self.error_recovery_client.send_goal(goal)
        # the error recovery may not response if the robot is not in error mode
        # and there is no reliable way to check if the robot is in error mode
        # so we just wait for a fixed time
        print('waiting for error recovery to finish...')
        self.error_recovery_client.wait_for_result()
        print('error recovery finished')

    def _check_initial_state(self):
        if self.latest_qpos is not None and not np.allclose(self.latest_qpos, [0, -1.3, 0, -2.5, 0, 1, 0], atol=1e-2):
            # raise ValueError("robot is not in initial state")
            print("robot is not in initial state:", self.latest_qpos)

            print("current eef pose:")
            print(self._get_eef_pose_from_ros())
            
        
        """ if self.gripper_status is not None and not self.gripper_status == "open":
            raise ValueError("gripper is not in initial state") """