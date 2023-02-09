#!/usr/bin/env python3

import numpy as np
import os
import rospy

from duckietown_msgs.msg import  WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped
from std_msgs.msg import Header, Float32
import message_filters

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType


class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # robot wheel radius
        self._radius = 0.0318

        # axis to wheel
        self._length = 0.05

        encLeft = f'/{self.veh_name}/left_wheel_encoder_node/tick'
        encRight= f'/{self.veh_name}/right_wheel_encoder_node/tick'
        encCMD = f'/{self.veh_name}/wheels_driver_node/wheels_cmd'
        twist = f'/{self.veh_name}/kinematics_node/velocity'

        self.left_tick = 0
        self.left_last_data = 0
        
        self.right_tick = 0
        self.right_last_data = 0

        self.dx_left = 0
        self.dx_right = 0

        self.robot_frame = {
            'x': 0,
            'y': 0,
            'theta': 0
        }

        self.circle_remain = 0

        self.stage = 0

        # Setup publishers
        self.pub_wheels_cmd = rospy.Publisher(encCMD, WheelsCmdStamped, queue_size=1)
        
        # Setup subscribers
        self.sub_encoder_ticks_left = message_filters.Subscriber(encLeft, WheelEncoderStamped)
        self.sub_encoder_ticks_right = message_filters.Subscriber(encRight, WheelEncoderStamped)
        self.sub_encoder_ticks = message_filters.ApproximateTimeSynchronizer([self.sub_encoder_ticks_left, self.sub_encoder_ticks_right], 10, 0.1, allow_headerless=True)
        self.sub_encoder_ticks.registerCallback(self.cb_encoder_data)

        #self.sub_executed_commands = rospy.Subscriber(encCMD, WheelsCmdStamped, self.cb_executed_commands)
        #self.sub_kinematics = rospy.Subscriber(twist, Twist2DStamped, self.update)

    
        self.log("Initialized")

    def cb_encoder_data(self, msgLeft, msgRight):
        """ Update encoder distance information from ticks.
        """
        if (self.left_last_data != 0):
            self.left_tick = msgLeft.data-self.left_last_data
        self.left_last_data = msgLeft.data
        self.dx_left = self.left_tick*self._radius*2*np.pi/msgLeft.resolution

        if (self.right_last_data != 0):
            self.right_tick = msgRight.data-self.right_last_data
        self.right_last_data = msgRight.data
        self.dx_right = self.right_tick*self._radius*2*np.pi/msgRight.resolution

        dA = (self.dx_left + self.dx_right)/2
        
        self.robot_frame['x'] += dA*np.cos(self.robot_frame['theta'])
        self.robot_frame['y'] += dA*np.sin(self.robot_frame['theta'])
        self.robot_frame['theta'] += (self.dx_right - self.dx_left)/(2*self._length)
        self.robot_frame['theta'] %= 2*np.pi

        self.log(str(self.robot_frame['x']) + "   " + str(self.robot_frame['y']) + "   " + str(self.robot_frame['theta']))
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msgLeft.header.stamp

        if (self.stage == 0):
            if (self.circle_remain == 0):
                self.circle_remain = 2*np.pi*0.7
            self.go_circle(msg_wheels_cmd)
        elif (self.stage == 1):
            self.move_backward(msg_wheels_cmd)
        elif (self.stage == 2):
            self.stop_being_silly(msg_wheels_cmd)

        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    # move to position relative to robot
    def move_forward(self, msg_wheels_cmd):
        if (self.robot_frame['x'] >= 1.23 and self.robot_frame['x'] <= 1.27):
                self.stage = 1
                msg_wheels_cmd.vel_right = 0
                msg_wheels_cmd.vel_left = 0
                return
        
        if (self.robot_frame['y'] > 0.01):
            msg_wheels_cmd.vel_right = 0.35
            msg_wheels_cmd.vel_left = 0.4

        elif (self.robot_frame['y'] < -0.01):
            msg_wheels_cmd.vel_right = 0.4
            msg_wheels_cmd.vel_left = 0.35

        else:
            msg_wheels_cmd.vel_right = 0.4
            msg_wheels_cmd.vel_left = 0.4

    def move_backward(self, msg_wheels_cmd):
        if (self.robot_frame['x'] >= -0.02 and self.robot_frame['x'] <= 0.01):
            self.stage = 2
            msg_wheels_cmd.vel_right = 0
            msg_wheels_cmd.vel_left = 0
            return
        
        if (self.robot_frame['y'] > 0.01):
            msg_wheels_cmd.vel_right = -0.35
            msg_wheels_cmd.vel_left = -0.4

        elif (self.robot_frame['y'] < -0.01):
            msg_wheels_cmd.vel_right = -0.4
            msg_wheels_cmd.vel_left = -0.35

        else:
            msg_wheels_cmd.vel_right = -0.4
            msg_wheels_cmd.vel_left = -0.4
        
    def stop_being_silly(self, msg_wheels_cmd):
        msg_wheels_cmd.vel_right = 0
        msg_wheels_cmd.vel_left = 0

    def rotate(self, msg_wheels_cmd):
        if (self.robot_frame['theta'] >= 3*np.pi/2-0.2 and self.robot_frame['theta'] <= 3*np.pi/2+0.2):
            self.stage = 1
            msg_wheels_cmd.vel_right = 0
            msg_wheels_cmd.vel_left = 0
            return

        msg_wheels_cmd.vel_right = -0.15
        msg_wheels_cmd.vel_left = 0.15

    def go_circle(self, msg_wheels_cmd):
        if (self.circle_remain >= -0.1 and self.circle_remain <= 0.2):
            self.stage = 2
            msg_wheels_cmd.vel_right = 0
            msg_wheels_cmd.vel_left = 0
            self.circle_remain = 0
            return
        
        msg_wheels_cmd.vel_right = 0.3
        msg_wheels_cmd.vel_left = 0.18
        self.circle_remain -= self.dx_right
        

    # https://docs.duckietown.org/daffy/duckietown-robotics-development/out/new_duckiebot_functionality.html
    def on_shutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""
        for i in range(5):
            msg_wheels_cmd = WheelsCmdStamped()
            msg_wheels_cmd.vel_right = 0
            msg_wheels_cmd.vel_left = 0
            self.pub_wheels_cmd.publish(msg_wheels_cmd)
        super(OdometryNode, self).on_shutdown()
        
        

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
