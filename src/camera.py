import rospy
import numpy as np
import cv2
import time
import zerorpc
import msgpack_numpy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import teleop.config as config
from teleop import gazebo_api as gzapi


class SimCamera(object):
    def __init__(self):
        self.config = config.simcam
        self.cv_bridge = CvBridge()
        rospy.loginfo('[monoclone:camera] start init sim cam')
        self.reset()
        rospy.loginfo('[monoclone:camera] done init sim cam')

    def reset(self):
        gzapi.respawn_model(self.config.model_name, self.config.model_path, self.config.init_pose)

    def get_frame(self):
        """
        Get a pair of depth and color frame from camera
        Image is in uint [R8G8B8] 3-channel
        Raw depth is in meters 1-channel
        """
        color_raw = rospy.wait_for_message('/camera/color/image_raw', Image)
        depth_raw = rospy.wait_for_message('/camera/depth/image_raw', Image)
        color_image = self.cv_bridge.imgmsg_to_cv2(color_raw, 'bgr8')
        depth_image = self.cv_bridge.imgmsg_to_cv2(depth_raw, '32FC1')

        # to 256 grayscale
        depth_256 = self.scale_to_255(depth_image, revert=False, as_int=True)

        # self.monitor(color_image, depth_256)

        # depth matrix should ascend its dimension by 1 (axis refers to 'image channel')
        # then we concat it to color matrix as if depth mat is the fourth channel of this image
        return np.concatenate((color_image, depth_256[:, :, np.newaxis]), axis=2)

    @staticmethod
    def scale_to_255(frame, revert=True, as_int=False):
        scaled_frame = (frame - np.min(frame)) * 255.0 / (np.max(frame) - np.min(frame))
        if revert:
            scaled_frame = 255 - scaled_frame
        return (np.rint(scaled_frame)).astype(np.uint8) if as_int else scaled_frame

    def monitor(self, color, depth):
        self.test(color)
        self.test2(depth)

    @staticmethod
    def test(msg):
        cv2.imshow('image_color', msg)
        cv2.waitKey(1)

    @staticmethod
    def test2(msg):
        cv2.imshow('image_depth', msg)


def patch():
    # enable ZeroRPC-msgpack to serialize numpy ndarray.
    # see: https://pypi.org/project/msgpack-numpy/
    #      https://github.com/0rpc/zerorpc-python/pull/187/
    msgpack_numpy.patch()
    rospy.loginfo('[monoclone:camera] money patch done')


def graceful_shutdown(server: zerorpc.Server):
    # ZeroRPC and rospy has a conflicting issue on exiting,
    # but it's ok to ignore these errors for all processes would eventually exit with a non-graceful manner.
    # see: https://www.cnblogs.com/flipped/p/15500530.html
    server.close()
    rospy.loginfo('[monoclone:camera] ZeroRPC Server exited.')


def main():
    patch()
    rospy.init_node('monoclone_ros_camera')
    cam_server = zerorpc.Server(SimCamera())
    cam_server.bind(f'tcp://0.0.0.0:{config.RPC_PORT_CAM}')

    # register shutdown hooks for zerorpc server to gracefully exit
    rospy.on_shutdown(lambda: graceful_shutdown(cam_server))
    rospy.loginfo('[monoclone:camera] Camera server starting...')
    cam_server.run()


if __name__ == '__main__':
    main()
