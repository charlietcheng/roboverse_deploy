import cv2
import numpy as np
from typing import Optional
from PIL import Image
import pyrealsense2 as rs
from sensor_msgs.msg import Image as ImageMsg
import rospy
from cv_bridge import CvBridge

class CameraRealsense:
    def __init__(self, serial_number: Optional[str] = None,
            use_infrared=False,
            width=640,
            height=360,
            fps=15
        ):
        context = rs.context()
        cameras = context.query_devices()
        for i, dev in enumerate(cameras):
            print(f"Device {i}: {dev}")
        print('all cameras listed')

        self.width = width
        self.height = height
        self.use_infrared = use_infrared
        # Configure depth and color streams
        self.config = rs.config()
        if serial_number is not None:
            self.config.enable_device(serial_number)
        self.config.enable_stream(
            rs.stream.depth, self.width, self.height, rs.format.z16, fps
        )
        self.config.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.rgb8, fps
        )
        if self.use_infrared:
            self.config.enable_stream(
                rs.stream.infrared, 1, self.width, self.height, rs.format.y8, fps
            )
            self.config.enable_stream(
                rs.stream.infrared, 2, self.width, self.height, rs.format.y8, fps
            )

        # Start RealSense pipeline
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.profile = self.pipeline.start(self.config)

        intrinsics_matrix = self.get_camera_intrinsics_matrix()
        self.fx = intrinsics_matrix[0, 0]
        self.fy = intrinsics_matrix[1, 1]
        self.cx = intrinsics_matrix[0, 2]
        self.cy = intrinsics_matrix[1, 2]

        self.cv_bridge = CvBridge()
        self.publisher = rospy.Publisher(f'/cameras/front', ImageMsg, queue_size=0)
        print("self.publisher: ", self.publisher)

        # try
        for _ in range(15):
            color_frame, depth_frame = self.capture_frame()
            Image.fromarray(color_frame).save("color.png")
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
            Image.fromarray(depth_colormap).save("depth.png")
            
            if np.sum(color_frame) > 0: # not black
                break

    def capture_frame(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        color_frame = np.asanyarray(frames.get_color_frame().get_data())
        depth_frame = (
            np.float32(np.asanyarray(frames.get_depth_frame().get_data())) / 1000.0
        )

        color_frame_pub = cv2.cvtColor(color_frame.copy(), cv2.COLOR_RGB2BGR)
        self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(color_frame_pub, "bgr8"))

        return color_frame, depth_frame

    def get_camera_intrinsics_matrix(self):
        color_intrin = self.get_camera_intrinsics()
        # Construct the intrinsics matrix
        intrinsics_matrix = np.array(
            [
                [color_intrin.fx, 0, color_intrin.ppx],
                [0, color_intrin.fy, color_intrin.ppy],
                [0, 0, 1],
            ]
        )
        return intrinsics_matrix

    def get_camera_intrinsics(self):
        color_intrin = (
            self.profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        return color_intrin

    def show_frame(self):
        color_frame, depth_frame = self.capture_frame()
        cv2.imshow("color_frame", cv2.cvtColor(color_frame, cv2.COLOR_RGB2BGR))
        cv2.imshow("depth_frame", depth_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            exit(0)

    def get_aligned_frames(self):
        # Wait for a coherent pair of frames: aligned depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = np.asanyarray(aligned_frames.get_color_frame().get_data())
        depth_frame = (
            np.float32(np.asanyarray(aligned_frames.get_depth_frame().get_data()))
            / 1000.0
        )
        if self.use_infrared:
            ir_l_image = np.asanyarray(aligned_frames.get_infrared_frame(1).get_data())
            ir_r_image = np.asanyarray(aligned_frames.get_infrared_frame(2).get_data())
        else:
            ir_l_image = None
            ir_r_image = None
        return color_frame, depth_frame, ir_l_image, ir_r_image


if __name__ == "__main__":
    
    rospy.init_node('realsense_camera', anonymous=True, disable_signals=True)

    # test camera
    camera = CameraRealsense()
    # camera.show_frame()
    idx = 0
    while True:
        color_frame, depth_frame, ir_l_image, ir_r_image = camera.get_aligned_frames()
        color_frame_pub = cv2.cvtColor(color_frame.copy(), cv2.COLOR_RGB2BGR)
        cv2.imshow("Color Frame", color_frame_pub)
        cv2.imshow("Depth Frame", depth_frame)
        if camera.use_infrared:
            cv2.imshow("IR Left Frame", ir_l_image)
            cv2.imshow("IR Right Frame", ir_r_image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        if idx == 0:
            Image.fromarray(color_frame).save(f"camera_{idx}.png")
        
        idx += 1

    rospy.signal_shutdown('done')