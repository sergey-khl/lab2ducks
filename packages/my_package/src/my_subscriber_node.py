#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo


class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        camTopic = '~/%s/camera_node/camera_info' % os.environ['VEHICLE_NAME']
        self.sub = rospy.Subscriber(camTopic, CameraInfo, self.callback)

    def callback(self, data):
        rospy.loginfo("height of %s, width of %s frame id: %s", data.height, data.width, data.header.frame_id)

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()