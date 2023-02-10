#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32
import message_filters


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
        self.stage = 0

        # Publishers
        self.pub_wheels_cmd = rospy.Publisher(encCMD, WheelsCmdStamped, queue_size=1)
        
        # Subscribing to the wheel encoders
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

        self.log(str(self.robot_frame['x']) + "   " + str(self.robot_frame['y']) + "   " + str(self.robot_frame['theta']))
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msgLeft.header.stamp

        #wait 
        if (self.stage == 0):
            self.stop_being_silly(msg_wheels_cmd, 1, 1/5)
        # box movement
        elif (self.stage == 1):
            self.rotate(msg_wheels_cmd, 2, 3*np.pi/2)
        elif (self.stage == 2):
            self.move_forward(msg_wheels_cmd, 3)
        elif (self.stage == 3):
            self.rotate(msg_wheels_cmd, 4, np.pi/2)
        elif (self.stage == 4):
            self.move_forward(msg_wheels_cmd, 5)
        elif (self.stage == 5):
            self.rotate(msg_wheels_cmd, 6, np.pi/2)
        elif (self.stage == 6):
            self.move_forward(msg_wheels_cmd, 7)
        #wait
        elif (self.stage == 7):
            self.stop_being_silly(msg_wheels_cmd, 8, 1/5)
        # go back to original pos
        elif (self.stage == 8):
            self.rotate(msg_wheels_cmd, 9, np.pi/2)
        elif (self.stage == 9):
            self.move_forward(msg_wheels_cmd, 10)
        # go back to original orientation
        elif (self.stage == 10):
            self.rotate(msg_wheels_cmd, 11, np.pi/2)
        elif (self.stage == 11):
            self.rotate(msg_wheels_cmd, 12, np.pi/2)
        #wait
	    elif (self.stage == 12):
      	    self.stop_being_silly(msg_wheels_cmd, 13, 1/5)
        # clockwise rotate
	    elif (self.stage == 13):
            self.go_circle(msg_wheels_cmd, 14)
            self.pub_wheels_cmd.publish(msg_wheels_cmd)

    # move to position relative to robot
    def move_forward(self, msg_wheels_cmd, next_stage):
        if (self.robot_frame['x'] >= 1.23 and self.robot_frame['x'] <= 1.27):
            self.stage = next_stage
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
        for i in range(3):
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