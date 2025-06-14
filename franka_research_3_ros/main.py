import rospy
import sys
# sys.path.append('./client')
# from vla_client.modes import GraspMode, GroundingMode, RecordMode, ReplayMode
import argparse
import time

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("dummpy", help="dummpy command")

# arg_parser.add_argument("--mode", default="grasp", choices=["grasp", "record", "grounding", "instruct", "replay"])
# arg_parser.add_argument("--controller", default="blocking", choices=["blocking", "non-blocking"])
arg_parser.add_argument("--camera", type=str, default=None, help="camera serial number")
# arg_parser.add_argument("--side-camera", type=str, default=None, help="side camera serial number, defaults to None")
arg_parser.add_argument("--save-to", type=str, help="directory for saving the experiment record, will auto add timestamp")
arg_parser.add_argument("--server-ip", type=str, default="localhost")
arg_parser.add_argument("--server-port", type=int, default=26784)
arg_parser.add_argument("--robot-ip", type=str, default="172.16.0.2")
arg_parser.add_argument("--agent", type=str, default="dp", choices=["dp", "openvla", "octo"])


def run(args):
    from src.robot import FrankaPanda
    from src.camera import CameraRealsense
    from src.agent import AgentDP, AgentOpenvla
    from src.visualize import RvizVisualizer
    from src.video_recorder import VideoRecoder
    from src.task import GraspTask

    viz = RvizVisualizer()
    video_recorder = VideoRecoder("test.mp4")
    camera = CameraRealsense() # None #
    robot = FrankaPanda() # None # 

    print("robot loaded")
    instruction = None
    if args.agent == "dp":
        agent = AgentDP()
        
    elif args.agent == "openvla":
        print("loading openvla agent...")
        agent = AgentOpenvla()
        print("openvla agent loaded")
        from src import input_typed
        user_input = input_typed('Pick up what? ', str)
        if user_input == '':
            return # quit the program

        instruction = f"Pick up {user_input}."
        
    elif args.agent == "octo":
        agent = AgentOcto()
    else:
        raise ValueError("agent not supported")
    
    print("agent: ", agent)
    task = GraspTask(agent, camera, robot)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # """ main loop: 
        #         1. read camera
        #         2. query action from server
        #         3. control franka 
        # """
        task.run(instruction)

        # viz.visualize(frame)
        # video_recorder.record(frame)

        rate.sleep()

if __name__ == "__main__":
    args = arg_parser.parse_args()
    rospy.init_node('franka_control', anonymous=True, disable_signals=True)
    try:
        run(args)
    except rospy.ROSInterruptException: 
        pass

    rospy.signal_shutdown('done')