#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class CameraSubscriberNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraSubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.veh_name = rospy.get_namespace().strip("/")

        # -- Publishers -- 
        self.pub_info = rospy.Publisher(
            '~path_to/image_info',
            String,
            queue_size=2,
        )
        self.pub_comp = rospy.Publisher(
            '~path_to/grayscale_image_compressed',
            CompressedImage,
            queue_size=2,
        )

        # -- Subscribers -- 
        self.sub_info = rospy.Subscriber(
            f'/{self.veh_name}/camera_node/camera_info',
            CameraInfo,
            self.callback_camera_info,
        )
        self.sub_comp = rospy.Subscriber(
            f'/{self.veh_name}/camera_node/image/camera_compressed',
            CompressedImage,
            self.callback_camera_compressed,
        )


    def callback_camera_info(self, img_info):
        # Create a string that represents the image dimensions in the format "height x width"
        img_dims = f"{img_info.height}x{img_info.width}"

        # Log the image dimensions to the ROS log
        rospy.loginfo(f"Image size: {img_dims}")

        # Publish the image dimensions to a ROS topic
        self.pub_info.publish(img_dims)


    def callback_camera_compressed(self, compressed):
        # Convert the compressed image data into a NumPy array of bytes
        raw_bytes = np.frombuffer(compressed.data, dtype=np.uint8)

        # Decode the image data into a OpenCV image
        cv_img = cv2.imdecode(raw_bytes, cv2.IMREAD_COLOR)
        
        # Convert the image from color to grayscale
        mono_cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        
        # Re-compress the image in JPEG format and convert it into a bytes object
        mono_raw = cv2.imencode('.jpeg', mono_cv_img)[1].tobytes()

        # Update the compressed image data with the new grayscale image data
        compressed.data = mono_raw
        
        # Publish the grayscale image data to a ROS topic
        self.pub_comp.publish(compressed)


if __name__ == '__main__':
    # create the node
    node = CameraSubscriberNode(node_name='compressed')
    # keep spinning
    rospy.spin()