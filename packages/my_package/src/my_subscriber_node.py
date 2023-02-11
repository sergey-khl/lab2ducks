#!/usr/bin/env python3

import numpy as np
import time
import rospy
import rosbag

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, LEDPattern
from std_msgs.msg import Header, String
from duckietown_msgs.srv import ChangePattern
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

        self.global_frame = {
            'x': 0.32,
            'y': 0.32,
            'theta': 0
        }

        self.dist_remain = 0
        self.ang_remain = 0
        self.stage = 0

        # Publishers
        self.pub_wheels_cmd = rospy.Publisher(encCMD, WheelsCmdStamped, queue_size=1)
        self.led = rospy.Publisher(f'/{self.veh_name}/led_emitter_node/led_pattern',
                                    LEDPattern, queue_size=1)
        
        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = message_filters.Subscriber(encLeft, WheelEncoderStamped)
        self.sub_encoder_ticks_right = message_filters.Subscriber(encRight, WheelEncoderStamped)
        self.sub_encoder_ticks = message_filters.ApproximateTimeSynchronizer([self.sub_encoder_ticks_left, self.sub_encoder_ticks_right], 10, 0.1, allow_headerless=True)
        self.sub_encoder_ticks.registerCallback(self.cb_encoder_data)

        # Proxy
        led_service = f'/{self.veh_name}/led_controller_node/led_pattern'
        rospy.wait_for_service(led_service)
        self.led_pattern = rospy.ServiceProxy(led_service, ChangePattern)

        # Bag
        self.bag_name = f'/{self.veh_name}/robot_data.bag'
        self.bag = rosbag.Bag(self.bag_name)

        self.change_led_lights('white')
    
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
        
        self.robot_frame['x'] = dA
        self.robot_frame['y'] = 0
        self.robot_frame['theta'] = (self.dx_right - self.dx_left)/(2*self._length)

        if self.ang_remain > 0:
            self.ang_remain -= np.abs(self.robot_frame['theta'])

        self.robot_frame['theta'] %= 2*np.pi

        self.global_frame['x'] += dA*np.cos(self.global_frame['theta'])
        self.global_frame['y'] += dA*np.sin(self.global_frame['theta'])
        self.global_frame['theta'] += (self.dx_right - self.dx_left)/(2*self._length)
        self.global_frame['theta'] %= 2*np.pi

        if self.dist_remain > 0:
            self.dist_remain -= self.dx_right

        # recording in the rosbag
        self.write_in_bag()


    def change_led_lights(self, color: str):
        '''
        Sends msg to service server
        Colors:
            "off": [0,0,0],
            "white": [1,1,1],
            "green": [0,1,0],
            "red": [1,0,0],
            "blue": [0,0,1],
            "yellow": [1,0.8,0],
            "purple": [1,0,1],
            "cyan": [0,1,1],
            "pink": [1,0,0.5],
        '''
        msg = String()
        msg.data = color
        self.led_pattern(msg)
        
        
    def cb_executed_commands(self, msg):
        self.left_dir = 1 if msg.vel_left > 0 else -1
        self.right_dir = 1 if msg.vel_right > 0 else -1


    def run(self, rate):
        self.change_led_lights('pink')

        while not self.is_shutdown:
            #self.log(str(self.robot_frame['x']) + "   " + str(self.robot_frame['y']) + "   " + str(self.robot_frame['theta']))
            self.log(str(self.global_frame['x']) + "   " + str(self.global_frame['y']) + "   " + str(self.global_frame['theta']))
            #wait 
            if (self.stage == 0):
                #self.do_nothing()
                self.stop_being_silly(1, 1/5)
                # box movement
            elif (self.stage == 1):
                self.rotate(2, 'cw')
            elif (self.stage == 2):
                self.move_forward(3)
            elif (self.stage == 3):
                self.rotate(4, 'ccw')

            elif (self.stage == 4):
                self.move_forward(5)
            elif (self.stage == 5):
                self.rotate(6, 'ccw')
            elif (self.stage == 6):
                self.move_forward(7)
                #wait
            elif (self.stage == 7):
                self.stop_being_silly(8, 1/5)
                # go back to original pos
            elif (self.stage == 8):
                self.rotate(9, 'ccw')
            elif (self.stage == 9):
                self.move_forward(10)
                # go back to original orientation
            elif (self.stage == 10):
                self.rotate(11, 'cw')
            elif (self.stage == 11):
                self.rotate(12, 'cw')
            elif (self.stage == 12):
                self.stop_being_silly(13, 1/5)
            elif (self.stage == 13):
                self.go_circle(14)
            rate.sleep()
            

    def write_in_bag(self):
        '''
        Writing in the bag the x and y coordinates of the robot.
        '''
        try:
            self.bag.write('x', self.robot_frame['x'])
            self.bag.write('y', self.robot_frame['y'])
        except Exception as e:
            print(f'This is the error message for bag: {e}')
            self.bag.close()


     # move to position relative to robot
    def move_forward(self, next_stage):
        self.log('going forward')
        msg_wheels_cmd = WheelsCmdStamped()
        if (self.dist_remain == 0):
            self.dist_remain = 1.25
        if (self.dist_remain >= -0.1 and self.dist_remain <= 0.2):
            self.stage = next_stage
            msg_wheels_cmd.vel_right = 0
            msg_wheels_cmd.vel_left = 0
            self.dist_remain = 0
            self.log('done moving forward')
        else:
            # if (self.robot_frame['y'] > 0.05):
            #     self.log('too left')
            #     msg_wheels_cmd.vel_right = 0.4
            #     msg_wheels_cmd.vel_left = 0.3
            # elif (self.robot_frame['y'] < -0.05):
            #     self.log('too right')
            #     msg_wheels_cmd.vel_right = 0.3
            #     msg_wheels_cmd.vel_left = 0.4
            # else:
            msg_wheels_cmd.vel_right = 0.4
            msg_wheels_cmd.vel_left = 0.4
        self.pub_wheels_cmd.publish(msg_wheels_cmd)


    def move_backward(self, next_stage):
        msg_wheels_cmd = WheelsCmdStamped()
        if (self.robot_frame['x'] >= -0.02 and self.robot_frame['x'] <= 0.01):
            self.stage = next_stage
            msg_wheels_cmd.vel_right = 0
            msg_wheels_cmd.vel_left = 0
            
        else:
            if (self.robot_frame['y'] > 0.01):
                msg_wheels_cmd.vel_right = -0.35
                msg_wheels_cmd.vel_left = -0.4

            elif (self.robot_frame['y'] < -0.01):
                msg_wheels_cmd.vel_right = -0.4
                msg_wheels_cmd.vel_left = -0.35

            else:
                msg_wheels_cmd.vel_right = -0.4
                msg_wheels_cmd.vel_left = -0.4
        self.pub_wheels_cmd.publish(msg_wheels_cmd)


    def do_nothing(self):
        pass
        

    def stop_being_silly(self, next_stage, hz):
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.vel_right = 0
        msg_wheels_cmd.vel_left = 0

        self.log('starting wait')
        rate = rospy.Rate(hz) # 1Hz
        rate.sleep()
        
        self.log('done wait')

        self.stage = next_stage
        self.pub_wheels_cmd.publish(msg_wheels_cmd)
        

    def rotate(self, next_stage, dir):
        self.log('start rotate')
        msg_wheels_cmd = WheelsCmdStamped()
        self.log(self.ang_remain)
        if (self.ang_remain == 0):
            self.ang_remain = np.pi/2
        if (self.ang_remain <= 0.15):
            self.stage = next_stage
            msg_wheels_cmd.vel_right = 0
            msg_wheels_cmd.vel_left = 0
            self.ang_remain = 0
            self.log('done rotating')
        else:
            if (dir == 'ccw'):
                msg_wheels_cmd.vel_right = 0.4
                msg_wheels_cmd.vel_left = -0.4
            else:
                msg_wheels_cmd.vel_right = -0.4
                msg_wheels_cmd.vel_left = 0.4
        self.pub_wheels_cmd.publish(msg_wheels_cmd)


    def go_circle(self, next_stage):
        self.log('start circle')
        msg_wheels_cmd = WheelsCmdStamped()
        if (self.dist_remain == 0):
            self.dist_remain = 2*np.pi*0.7
        if (self.dist_remain >= -0.1 and self.dist_remain <= 0.2):
            self.stage = next_stage
            msg_wheels_cmd.vel_right = 0
            msg_wheels_cmd.vel_left = 0
            self.dist_remain = 0
            self.log('done with circle')
        else:
            msg_wheels_cmd.vel_right = 0.3
            msg_wheels_cmd.vel_left = 0.5
            
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

        
    def read_from_bag(self):
        for topic, msg, t in self.bag.read_messages(topics=['x', 'y']):
            print('Rosbag data:', topic, msg, t)
        self.bag.close()

    # https://docs.duckietown.org/daffy/duckietown-robotics-development/out/new_duckiebot_functionality.html
    def on_shutdown(self):
        """Shutdown procedure.
        Publishes a zero velocity command at shutdown."""
        try:
            for i in range(15):
                msg_wheels_cmd = WheelsCmdStamped()
                msg_wheels_cmd.vel_right = 0
                msg_wheels_cmd.vel_left = 0
                self.pub_wheels_cmd.publish(msg_wheels_cmd)
        except:
            super(OdometryNode, self).on_shutdown()
        
        

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    rate = rospy.Rate(30)
    node.run(rate)

    #rospy.spin()

    # # Keep it spinning to keep the node alive
    
    # #rospy.spin()
    # try:
    #     node.run(rate)
    # except rospy.ROSInterruptException:
    #     pass