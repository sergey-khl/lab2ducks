#!/usr/bin/env python3

import numpy as np
import time
import rospy
import rosbag

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, LEDPattern, Twist2DStamped
from std_msgs.msg import String, Float64
from duckietown_msgs.srv import ChangePattern
import message_filters
from pathlib import Path


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
        twist = f'/{self.veh_name}/car_cmd_switch_node/cmd'

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

        self.global_frame = {
            'x': 0,
            'y': 0,
            'theta': 0
        }

        self.dist_remain = 0
        self.ang_remain = 0
        self.stage = 0

        # -- Publishers -- 
        self.pub_wheels_cmd = rospy.Publisher(
            f'/{self.veh_name}/wheels_driver_node/wheels_cmd', 
            WheelsCmdStamped, 
            queue_size=1
        )
        self.led = rospy.Publisher(
            f'/{self.veh_name}/led_emitter_node/led_pattern',
            LEDPattern,
            queue_size=1
        )
        self.pub_twist = rospy.Publisher(
            twist, 
            Twist2DStamped, 
            queue_size=1
        )
        
        # -- Subscribers --
        self.sub_encoder_ticks_left = message_filters.Subscriber(
            f'/{self.veh_name}/left_wheel_encoder_node/tick', 
            WheelEncoderStamped
        )
        self.sub_encoder_ticks_right = message_filters.Subscriber(
            f'/{self.veh_name}/right_wheel_encoder_node/tick', 
            WheelEncoderStamped
        )

        self.sub_encoder_ticks = message_filters.ApproximateTimeSynchronizer(
            [self.sub_encoder_ticks_left, self.sub_encoder_ticks_right], 10, 0.1, allow_headerless=True)
        self.sub_encoder_ticks.registerCallback(self.cb_encoder_data)

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
        

        if self.ang_remain != 0:
            self.ang_remain -= np.abs((self.dx_right - self.dx_left)/(2*self._length))



        self.robot_frame['x'] += dA*np.cos(self.global_frame['theta'])
        self.robot_frame['y'] += dA*np.sin(self.global_frame['theta'])
        self.robot_frame['theta'] += (self.dx_right - self.dx_left)/(2*self._length)
        self.robot_frame['theta'] %= 2*np.pi

        if self.dist_remain != 0:
            self.dist_remain -= np.abs(dA)

        # update global coordinates
        self.initial_to_global()

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
        start = time.time()

        while not self.is_shutdown:
            self.log(str(self.robot_frame['x']) + "   " + str(self.robot_frame['y']) + "   " + str(self.robot_frame['theta']))
            #self.log(str(self.global_frame['x']) + "   " + str(self.global_frame['y']) + "   " + str(self.global_frame['theta']))
            #wait 
            if (self.stage == 0):
                
                self.change_led_lights('cyan')
                self.stop_being_silly(1, 1/5)
                #self.rotate(-1, 'cw')
            # box movement
            elif (self.stage == 1):
                self.change_led_lights('green')
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
                self.change_led_lights('cyan')
                self.left_dir = 0
                self.right_dir = 0
                self.stop_being_silly(8, 1/5)
            # go back to original pos
            elif (self.stage == 8):
                self.change_led_lights('purple')
                self.left_dir = -1
                self.right_dir = 1
                self.rotate(9, 'ccw')
            elif (self.stage == 9):
                self.left_dir = 1
                self.right_dir = 1
                self.move_forward(10)
            # go back to original orientation
            elif (self.stage == 10):
                self.rotate(11, 'cw')
            elif (self.stage == 11):
                self.rotate(12, 'cw')
            elif (self.stage == 12):
                self.change_led_lights('cyan')
                self.stop_being_silly(13, 1/5)
            elif (self.stage == 13):
                self.change_led_lights('yellow')
                self.left_dir = 1
                self.right_dir = 1
                self.go_circle(14)

            else:
                end = time.time()
                self.log('total execution time: ' + str(end-start))
                self.read_from_bag()

                return
            rate.sleep()
        
            

    def write_in_bag(self):
        '''
        Writing in the bag the x and y coordinates of the robot.
        '''
        try:
            x = Float64()
            x.data = self.global_frame['x']
            y = Float64()
            y.data = self.global_frame['y']
            theta = Float64()
            theta.data = self.global_frame['theta']

            self.bag.write('x', x)
            self.bag.write('y', y)
            self.bag.write('theta', theta)
        except Exception as e:
            print(f'This is the error message for bag: {e}')
            self.bag.close()

    def move (self, v, omega):
        # move using twist by sending velocity and omega
        twist = Twist2DStamped()
        twist.v = v
        twist.omega = omega
        self.pub_twist.publish(twist)


     # move to position relative to robot
    def move_forward(self, next_stage):
        self.log('going forward')
        if (self.dist_remain == 0):
            self.dist_remain = 1.25
        if (self.dist_remain <= 0.2):
            self.stage = next_stage
            self.move(0, 0)
            self.dist_remain = 0
            self.log('done moving forward')
        else:
            self.move(0.35, 0.5)



    def do_nothing(self):
        pass
        

    def stop_being_silly(self, next_stage, hz):
    
        self.move(0, 0)

        self.log('starting wait')
        rate = rospy.Rate(hz) # 1Hz
        rate.sleep()
        
        self.log('done wait')

        self.stage = next_stage

    def correct(self, ang, dir):
        speed = np.sign(ang)*dir*9
        self.move(0, speed)
        hz = np.clip([int(1/np.abs(ang))], 5, 20)
        rate = rospy.Rate(hz[0])
        rate.sleep()
        

    def rotate(self, next_stage, dir):
        self.log('start rotate')
        if (self.ang_remain == 0):
            self.ang_remain = np.pi/2
        if (self.ang_remain <= 0.5):
            while (np.abs(self.ang_remain) > 0.05):
                self.stop_being_silly(next_stage, 5)
                self.correct(self.ang_remain, 1) if dir == 'ccw' else self.correct(self.ang_remain, -1)
                self.stop_being_silly(next_stage, 5)
                self.ang_remain = 0
            self.log('done rotating')
        else:
            if (dir == 'ccw'):
                self.move(0, 9)
            else:
                self.move(0, -9)
        


    def go_circle(self, next_stage):
        self.log('start circle')
        if (self.dist_remain == 0):
            self.dist_remain = 2*np.pi*0.3
        if (self.dist_remain >= -0.1 and self.dist_remain <= 0.2):
            self.stage = next_stage
            self.stop_being_silly(next_stage, 10)
            self.dist_remain = 0
            self.log('done with circle')
        else:
            self.move(0.4, -2.5)
            

    def initial_to_global(self):
        initial_frame_cord = np.array([self.robot_frame['x'], self.robot_frame['y'], 1]).transpose()
        transformation_mat = np.array([[0, -1, 0.32],[1, 0, -0.32],[0, 0, 1]])
        global_frame_angle = (self.robot_frame['theta'] + np.pi/2) % 2*np.pi
        global_frame_cord = transformation_mat @ initial_frame_cord
        self.global_frame['x'] = global_frame_cord[0]
        self.global_frame['y'] = global_frame_cord[1]
        self.global_frame['theta'] = global_frame_angle
        
    def read_from_bag(self):
        try:
            for topic, msg, t in self.bag.read_messages(topics=['x', 'y', 'theta']):
                print('Rosbag data:', topic, msg, t)
            
        except Exception as e:
            print(e)
            
            self.log('done reading bag')

    

    # https://docs.duckietown.org/daffy/duckietown-robotics-development/out/new_duckiebot_functionality.html
    def on_shutdown(self):
        """Shutdown procedure.
        Publishes a zero velocity command at shutdown."""
        try:
            for i in range(15):
                
                self.move(0, 0)
            
        except Exception as e:
            print(e)
            
            super(OdometryNode, self).on_shutdown()
        self.read_from_bag()
        self.bag.close()
        
        
        

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    rate = rospy.Rate(30)
    node.run(rate)
    node.read_from_bag()

    node.bag.close()
    