import rospy
import sys
from src.robot import FrankaPanda
from src.camera import CameraRealsense
def run():

    robot = FrankaPanda()
    # robot.open_gripper()
    robot.close_gripper()

    camera = CameraRealsense()
    camera.capture_frame()

if __name__ == "__main__":
    rospy.init_node('open_gripper_node', anonymous=True)

    try:
        run()
    except rospy.ROSInterruptException: 
        pass

    rospy.signal_shutdown('done')