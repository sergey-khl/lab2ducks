#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        camTopic = '~/%s/camera_node/image/compressed' % os.environ['VEHICLE_NAME']
        pubTopic = '~compressed'
        self.img = CompressedImage()
        self.sub = rospy.Subscriber(camTopic, CompressedImage, self.callback)

        self.pub = rospy.Publisher(pubTopic, CompressedImage, queue_size=10)
        

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            self.pub.publish(self.img)
            rate.sleep()

    def callback(self, data):
        self.img.data = data.data

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()