#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        camTopic = '~camera_node/img/compressed'
        self.sub = rospy.Subscriber(camTopic, CompressedImage, self.callback)

    def callback(self, data):
        rospy.loginfo("img: ", data.data)

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='compressed')
    # keep spinning
    rospy.spin()