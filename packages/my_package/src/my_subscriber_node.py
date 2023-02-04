#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        encLeft = f'/{self.veh_name}/left_wheel_encoder_node/tick'
        encRight= f'/{self.veh_name}/right_wheel_encoder_node/tick'
        encCMD = f'/{self.veh_name}/wheels_driver_node/wheels_cmd'
        twist = f'/{self.veh_name}/kinematics_node/velocity'

        self.left_tick = 0
        self.left_dir = 0
        self.left_last_data = 0
        
        self.right_tick = 0
        self.right_dir = 0
        self.right_last_data = 0

        self.dx_left = 0
        self.dx_right = 0
        
        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(encLeft, WheelEncoderStamped, lambda x: self.cb_encoder_data('left', x))
        self.sub_encoder_ticks_right = rospy.Subscriber(encRight, WheelEncoderStamped, lambda x: self.cb_encoder_data('right', x))
        self.sub_executed_commands = rospy.Subscriber(encCMD, WheelsCmdStamped, self.cb_executed_commands)
        #self.sub_executed_commands = rospy.Subscriber(twist, Twist2DStamped, self.cb_executed_commands)

        # Publishers
        #self.pub_integrated_distance_left = rospy.Publisher(...)
        #self.pub_integrated_distance_right = rospy.Publisher(...)

        self.log("Initialized")

    def cb_encoder_data(self, wheel, msg):
        """ Update encoder distance information from ticks.
        """
        if (wheel == 'left'):
            self.left_tick += (msg.data-self.left_last_data)*self.left_dir
            self.left_last_data = msg.data
            self.dx_left = self.left_tick*self._radius*2*3.14/msg.resolution
        elif (wheel == 'right'):
            self.right_tick += (msg.data-self.right_last_data)*self.right_dir
            self.right_last_data = msg.data
            self.dx_right = self.right_tick*self._radius*2*3.14/msg.resolution
        self.log(self._radius)
        #self.log("right" + str(self.dx_right) + "left" + str(self.dx_left))
        


    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        #self.log(msg)
        if (msg.vel_left > 0):
            self.left_dir = 1
        elif (msg.vel_left < 0):
            self.log("ISUDFIUSDFIUSDBFIUSDBF")
            self.left_dir = -1
        elif (msg.vel_left == 0):
            self.left_dir = 0

        if (msg.vel_right > 0):
            self.right_dir = 1
        elif (msg.vel_right < 0):
            self.right_dir = -1
        elif (msg.vel_right == 0):
            self.right_dir = 0
        #self.log(self.right_dir)

        
        

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")