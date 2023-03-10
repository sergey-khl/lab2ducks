#!/usr/bin/env python3

import numpy as np
import time
import rospy
import rosbag

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped, LEDPattern, WheelsCmdStamped
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from duckietown_msgs.srv import ChangePattern
import message_filters
from pathlib import Path


class GoRobot(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(GoRobot, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # robot wheel radius
        self._radius = 0.0318
        # axis to wheel
        self._length = 0.05
        
        twist = f'/{self.veh_name}/car_cmd_switch_node/cmd'
        kin = f'/{self.veh_name}/kinematics_node/velocity'
        pose = f'/{self.veh_name}/deadreckoning_node/odom'
        encCMD = f'/{self.veh_name}/wheels_driver_node/wheels_cmd'

        self.left_tick = 0
        self.left_last_data = 0
        self.left_dir = 0
        
        self.right_tick = 0
        self.right_last_data = 0
        self.right_dir = 0

        self.dx_left = 0
        self.dx_right = 0

        self.robot_frame = {
            'x': 0,
            'y': 0,
            'theta': 0
        }

# change back to .32 .32
        self.global_frame = {
            'x': 0,
            'y': 0,
            'theta': 0
        }

        self.prev_pos = Odometry()
        self.first_message = True
        self.orig_x = 0
        self.orig_y = 0

        self.dist_remain = 0
        self.ang_remain = 0
        self.stage = 0

        # -- Publishers -- 
        self.pub_twist = rospy.Publisher(
            twist, 
            Twist2DStamped, 
            queue_size=1
        )
        self.pub_kin = rospy.Publisher(
            kin, 
            Twist2DStamped,
            queue_size=1
        )
        self.led = rospy.Publisher(
            f'/{self.veh_name}/led_emitter_node/led_pattern',
            LEDPattern,
            queue_size=1
        )
        #self.pub_wheels_cmd = rospy.Publisher(encCMD, WheelsCmdStamped, queue_size=1)
        
        # -- Subscribers --
        self.sub_pose = rospy.Subscriber(
            pose, 
            Odometry,
            self.cb_encoder_data,
            queue_size=10
        )

        # -- Proxy -- 
        led_service = f'/{self.veh_name}/led_controller_node/led_pattern'
        rospy.wait_for_service(led_service)
        self.led_pattern = rospy.ServiceProxy(led_service, ChangePattern)

        # -- ROS Bag -- (https://codeberg.org/akemi/duckietown/src/commit/70507322806ae0ff4e39fcbfa4bada3a7328a179/lab2/heartbeat-ros/packages/odometry_node/src/odometry_publisher_node.py)
        bag_name = time.ctime().replace(' ', '_').replace(':', '-')
        bag_filename = f'/data/bags/odometry_at_{bag_name}.bag'
        Path(bag_filename).parent.mkdir(parents=True, exist_ok=True)
        self.bag = rosbag.Bag(bag_filename, 'w')
        rospy.loginfo(f"Made a bag {self.bag}")

        self.change_led_lights('white')
    
        self.log("Initialized")


    def cb_encoder_data(self, msg):
        """ Update encoder distance information from ticks.
        """
        if self.first_message:
            self.first_message = False
            self.orig_x = msg.pose.pose.position.x
            self.orig_y = msg.pose.pose.position.y
            return
            
        time = (msg.header.stamp - self.prev_pos.header.stamp).to_sec()
        self.global_frame['x'] += msg.pose.pose.position.x - self.prev_pos.pose.pose.position.x
        self.global_frame['y'] += msg.pose.pose.position.y - self.prev_pos.pose.pose.position.y
        self.global_frame['theta'] += msg.twist.twist.angular.z*time
        self.global_frame['theta'] %= 2*np.pi
        #     self.left_tick = msgLeft.data-self.left_last_data
        # self.left_last_data = msgLeft.data
        # self.dx_left = self.left_dir*self.left_tick*self._radius*2*np.pi/msgLeft.resolution

        # if (self.right_last_data != 0):
        #     self.right_tick = msgRight.data-self.right_last_data
        # self.right_last_data = msgRight.data
        # self.dx_right = self.right_dir*self.right_tick*self._radius*2*np.pi/msgRight.resolution
        
        # dA = (self.dx_left + self.dx_right)/2
        
        # self.robot_frame['x'] = dA
        # self.robot_frame['y'] = 0
        # self.robot_frame['theta'] = (self.dx_right - self.dx_left)/(2*self._length)

        if self.ang_remain > 0:
            self.ang_remain -= np.abs(msg.twist.twist.angular.z*time)

        # self.robot_frame['theta'] %= 2*np.pi

        # self.global_frame['x'] += dA*np.cos(self.global_frame['theta'])
        # self.global_frame['y'] += dA*np.sin(self.global_frame['theta'])
        # self.global_frame['theta'] += (self.dx_right - self.dx_left)/(2*self._length)
        # self.global_frame['theta'] %= 2*np.pi

        if self.dist_remain > 0:
            self.dist_remain -= np.sqrt((msg.pose.pose.position.x - self.prev_pos.pose.pose.position.x)**2+(msg.pose.pose.position.y - self.prev_pos.pose.pose.position.y)**2)

        # # recording in the rosbag
        # self.write_in_bag()
        self.prev_pos = msg
        


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
                self.move_forward(-1)
                #self.stop_being_silly(1, 1/5)
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
            x = Float64()
            x.data = self.robot_frame['x']
            y = Float64()
            y.data = self.robot_frame['y']

            self.bag.write('robot_frameX', x)
            self.bag.write('robot_frameY', y)
        except Exception as e:
            print(f'This is the error message for bag: {e}')
            self.bag.close()


     # move to position relative to robot
    def move_forward(self, next_stage):
        self.log('going forward')
        twist = Twist2DStamped()
        if (self.dist_remain == 0):
            self.dist_remain = 1.25
        if (self.dist_remain >= -0.1 and self.dist_remain <= 0.2):
            self.stage = next_stage
            twist.v = 0
            twist.omega = 0
            self.dist_remain = 0
            self.log('done moving forward')
        else:
            if (self.global_frame['y'] > 0.05):
                self.log('too left')
                twist.v = 0.45
                twist.omega = -2
            elif (self.global_frame['y'] < -0.05):
                self.log('too right')
                twist.v = 0.45
                twist.omega = 2
            else:
                twist.v = .5
                twist.omega = 0
        self.pub_twist.publish(twist)


    def move_backward(self, next_stage):
        twist = WheelsCmdStamped()
        if (self.robot_frame['x'] >= -0.02 and self.robot_frame['x'] <= 0.01):
            self.stage = next_stage
            twist.vel_left = 0
            twist.vel_right = 0
            
        else:
            if (self.robot_frame['x'] > 0.0):
                twist.vel_left = .5
                twist.vel_right = -3

            elif (self.robot_frame['x'] < -0.01):
                twist.vel_left = .5
                twist.vel_right = 3

            else:
                twist.vel_left = .5
                twist.vel_right = 0
        self.pub_twist.publish(twist)


    def do_nothing(self):
        pass
        

    def stop_being_silly(self, next_stage, hz):
        twist = Twist2DStamped()
        twist.v = 0
        twist.omega = 0

        self.log('starting wait')
        rate = rospy.Rate(hz) # 1Hz
        rate.sleep()
        
        self.log('done wait')

        self.stage = next_stage
        self.pub_twist.publish(twist)
        

    def rotate(self, next_stage, dir):
        #self.log('start rotate')
        twist = Twist2DStamped()
        #self.log(self.ang_remain)
        if (self.ang_remain == 0):
            self.ang_remain = np.pi/2
        if (self.ang_remain <= 0.15):
            self.stage = next_stage
            twist.v = 0
            twist.omega = 0
            self.ang_remain = 0
            #self.log('done rotating')
        else:
            if (dir == 'ccw'):
                twist.v = 0
                twist.omega = 10
            else:
                twist.v = 0
                twist.omega = -10
        self.pub_twist.publish(twist)
        self.pub_kin.publish(twist)


    def go_circle(self, next_stage):
        self.log('start circle')
        twist = Twist2DStamped()
                
        if (self.dist_remain == 0):
            self.dist_remain = 2*np.pi*0.7
        if (self.dist_remain >= -0.1 and self.dist_remain <= 0.2):
            self.stage = next_stage
            twist.v = 0
            twist.omega = 0
            self.dist_remain = 0
            self.log('done with circle')
        else:
            twist.v = .4
            twist.omega = 5
            
        self.pub_twist.publish(twist)

        
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
                twist = Twist2DStamped()
                twist.v = 0
                twist.omega = 0
                self.pub_twist.publish(twist)
        except:
            super(GoRobot, self).on_shutdown()
        
        

if __name__ == '__main__':
    node = GoRobot(node_name='twisting_node')
    rate = rospy.Rate(30)
    node.run(rate)

    #rospy.spin()

    # # Keep it spinning to keep the node alive
    
    # #rospy.spin()
    # try:
    #     node.run(rate)
    # except rospy.ROSInterruptException:
    #     pass