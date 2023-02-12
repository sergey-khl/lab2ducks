#!/usr/bin/env python3
import numpy as np
import time
import rospy
import rosbag

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, LEDPattern, Twist2DStamped
from std_msgs.msg import Header, String, Float64
from duckietown_msgs.srv import ChangePattern
from pathlib import Path

# References: https://github.com/anna-ssi/duckiebot/blob/50d0b24eab13eb32d92fa83273a05564ca4dd8ef/assignment2/src/wheel_odometry.py


class BasicMovemenNode(DTROS):

    def __init__(self, node_name: str, desired_distance: float = 1.25):

        # Initialize the DTROS parent class
        super(BasicMovemenNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION)
        
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/radius')
        self._baseline = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/baseline')

        # -- Assigning variables -- 
        self.desired_distance = desired_distance # distance you want to travel (m)
        self.prev_values = {'left': 0, 'right': 0} # wheel encoder value at time t-1
        self.d = {'left': 0, 'right': 0} # distance traveled between time t-1 and t
        self.traveled_distance = {'left': 0, 'right': 0} # total distance traveled by each wheel
        #self.robot_frame = {'x': 0, 'y': 0, 'theta': np.pi/2} # (x,y) translation and orientation of robot in its frame
        self.robot_frame = {'x': 0, 'y': 0, 'theta': 0} # (x,y) translation and orientation of robot in its frame
        self.global_frame = {'x': 0, 'y': 0, 'theta': 0} # (x,y) translation and orientation of robot in world frame 

        self.dir = {'left': 0, 'right': 0} # direction we are currently moving in

        # -- Subscribers -- 
        self.sub_encoder_ticks_left = rospy.Subscriber(
            f'/{self.veh_name}/left_wheel_encoder_node/tick',
            WheelEncoderStamped, 
            self.cb_encoder_data,
            callback_args='left', 
            queue_size=1
        )
        self.sub_encoder_ticks_right = rospy.Subscriber(
            f'/{self.veh_name}/right_wheel_encoder_node/tick',
            WheelEncoderStamped, 
            self.cb_encoder_data,
            callback_args='right', 
            queue_size=1
        )

        # -- Publishers -- 
        self.pub_wheel_commands = rospy.Publisher(
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
            f'/{self.veh_name}/car_cmd_switch_node/cmd', 
            Twist2DStamped, 
            queue_size=1
        )

        # -- Proxy -- 
        led_service = f'/{self.veh_name}/led_controller_node/led_pattern'
        rospy.wait_for_service(led_service)
        self.led_pattern = rospy.ServiceProxy(led_service, ChangePattern)

        # -- ROS Bag -- 
        # Reference: (https://codeberg.org/akemi/duckietown/src/commit/70507322806ae0ff4e39fcbfa4bada3a7328a179/lab2/heartbeat-ros/packages/odometry_node/src/odometry_publisher_node.py)
        bag_name = time.ctime().replace(' ', '_').replace(':', '-')
        bag_filename = f'/data/bags/odometry_at_{bag_name}.bag'
        Path(bag_filename).parent.mkdir(parents=True, exist_ok=True)
        self.bag = rosbag.Bag(bag_filename, 'w')
        rospy.loginfo(f"Made a bag {self.bag}")


    def cb_encoder_data(self, msg: WheelEncoderStamped, wheel: str):
        '''
        Getting the data from the wheel encoder node 
        Args:
            msg: the latest data from the subscriber
            wheel: an argument to know which wheel is in use
        '''

        # init on start  of new cmd
        if self.prev_values[wheel] == 0:
            self.prev_values[wheel] = msg.data
            return
        
        # find wheel rotation by encoder difference
        diff_value = msg.data - self.prev_values[wheel]
        self.prev_values[wheel] = msg.data


        #rospy.loginfo(wheel + ' tick ' + str(msg.data))

        # caluclates the distance travled
        dist = 2 * np.pi * self._radius * diff_value / msg.resolution

        # update distance travled 
        self.d[wheel] = dist * self.dir[wheel]
        self.traveled_distance[wheel] += dist
        #rospy.loginfo(wheel+ ' d ' + str(self.traveled_distance[wheel]))

        # updating the robot frame coordinates & the world frame coordinates
        self.update_coordinates()


    def clear(self):
        '''
        Clearing the traveled distance after every command.
        '''
        self.traveled_distance = {'left': 0, 'right': 0}


    def move(self, vel_left: float = 0.0, vel_right: float = 0.0):
        '''
        Updating the velocity of the robot
        Args:
            vel_left: left wheel velocity
            vel_right: right wheel velocity
        '''
        if (vel_left > 0): self.dir['left'] = 1
        elif (vel_left < 0): self.dir['left'] = -1
        else: self.dir['left'] = 0 

        if (vel_right > 0): self.dir['right'] = 1
        elif (vel_right < 0): self.dir['right'] = -1
        else: self.dir['right'] = 0 


        header = Header()
        self.pub_wheel_commands.publish(
            WheelsCmdStamped(
                header=header,
                vel_left=vel_left,
                vel_right=vel_right
            ))
            
            

    def stop(self, seconds: int = None):
        '''
        Sends a velocity of 0 to each wheel of the duckiebot to stop moving
        Args:
            seconds: seconds duckiebot should stop
        '''

        # changing the LED light to red
        self.change_led_lights('red')

        if seconds is None:
            self.move(vel_left=0.0, vel_right=0.0)
            return

        start = time.time()
        end = start

        while not rospy.is_shutdown() and (end - start) < seconds:
            self.move(vel_left=0.0, vel_right=0.0)
            end = time.time()


    def rotate(self, rate: rospy.Rate, desired_distance: float, vel_left: float = 0.2,
               vel_right: float = -0.2, clockwise: bool = True):
        '''
        Rotating by the specified distance.
        Args:
            rate: an instance of rospy.Rate
            desired_distance: the desired distance the duckiebot should travel
            vel_left: left wheel velocity
            vel_right: right wheel velocity
            clockwise: and indicator whether the turn is in clockwise direction or not
        '''

        # changing the LED color to blue and clearing the traveled disctance
        self.change_led_lights('blue')
        self.clear()

        # TODO: having only one condition helps?
        #  (self.traveled_distance['left'] < desired_distance and
        #    self.traveled_distance['right'] > -desired_distance)
        if clockwise:
            while not rospy.is_shutdown() and self.traveled_distance['left'] < desired_distance:
                self.move(vel_left=vel_left, vel_right=vel_right)
                rate.sleep()
        else:
            while not rospy.is_shutdown() and self.traveled_distance['right'] < desired_distance:
                self.move(vel_left=vel_left, vel_right=vel_right)
                rate.sleep()



    def forward(self, rate: rospy.Rate, desired_distance: float, vel_left: int = 0.43, vel_right: int = 0.42):
        '''
        Going forward by the specified distance & speed 
        Args:
            rate: an instance of rospy.Rate
            desired_distance: the desired distance the duckiebot should travel
            vel_left: left wheel velocity
            vel_right: right wheel velocity
        '''

        # changing the LED color to green and clearing the traveled disctance
        self.change_led_lights('green')
        self.clear()

        while not rospy.is_shutdown() and (self.traveled_distance['left'] < desired_distance and
                                           self.traveled_distance['right'] < desired_distance):
            self.move(vel_left=vel_left, vel_right=vel_right)
            rate.sleep()


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


    def update_coordinates(self):
        '''
        Updating the robot frame coordinates and the world coordinates then write them in a bag
        '''
        # update robot frame
        delta_theta = (self.d['right'] - self.d['left']) / (self._baseline)
        delta_A = (self.d['left'] + self.d['right']) / 2
        delta_x = delta_A * np.cos(self.robot_frame['theta'])
        delta_y = delta_A * np.sin(self.robot_frame['theta'])

        self.robot_frame['x'] += delta_x
        self.robot_frame['y'] += delta_y
        self.robot_frame['theta'] = (
            self.robot_frame['theta'] + delta_theta) % (2 * np.pi)
        
        # update global frame
        self.initial_to_global()

        # recording in the rosbag
        self.write_in_bag()


    def write_in_bag(self):
        '''
        Writing in the bag the x and y coordinates of the robot.
        '''
        #print(self.global_frame['x'], self.global_frame['y'], self.global_frame['theta'])
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
            #self.bag.write('orientation: ', theta)

        except Exception as e:
            print(f'This is the error message for bag: {e}')
            self.bag.close()


    def read_from_bag(self): 
        for topic, msg, t in self.bag.read_messages(topics=['x', 'y', 'theta']):
            print('Rosbag data:', topic, msg, t)
        self.bag.close()


    def initial_to_global(self):
        initial_frame_cord = np.array([self.robot_frame['x'], self.robot_frame['y'], 1]).transpose()
        transformation_mat = np.array([[0, -1, 0.32],[1, 0, -0.32],[0, 0, 1]])
        global_frame_angle = (self.robot_frame['theta'] + np.pi/2) % 2*np.pi
        global_frame_cord = transformation_mat @ initial_frame_cord
        self.global_frame['x'] = global_frame_cord[0]
        self.global_frame['y'] = global_frame_cord[1]
        self.global_frame['theta'] = global_frame_angle

        rospy.loginfo(self.robot_frame)
        rospy.loginfo(self.global_frame)


    def run(self):
        '''
        The main running point of the program
        '''
        # publish message every 0.1 second
        # TODO: change the rate value to see if it has any effect
        rate = rospy.Rate(30)

        # stop for 5 seconds
        # self.stop(seconds=2)

        # turning clockwise
        #dis_rot_distance = np.pi * self._baseline / 2
        # angle_fix = np.deg2rad(10)
        # dis_rot_distance = (2*np.pi - 4*angle_fix)*(self._baseline/2) / 4 

        # self.rotate(rate, dis_rot_distance, vel_left=0.6, vel_right=0)
        # self.stop(0.2)

        # # TODO: does removing stop() mess up with the travelling
        #

        # move forward
        self.desired_distance = 0.199805
        self.forward(rate, self.desired_distance, vel_left=0.4, vel_right=0.4)
        self.stop(0.2)

        # # turning counter-clockwise
        # self.rotate(rate, dis_rot_distance, vel_left=0.45,
        #             vel_right=-0.45, clockwise=False)
        # self.stop(0.2)

        # move forward
        # self.forward(rate, self.desired_distance, vel_left=0.4, vel_right=0.42)
        # self.stop()

        # turning counter-clockwise
        # self.rotate(rate, dis_rot_distance, vel_left=0,
        #             vel_right=0.3, clockwise=False)
        # self.stop()

        # move forward
        # self.forward(rate, self.desired_distance, vel_left=0.4, vel_right=0.42)

        # self.stop()

        # getting back to initial position
        # turning counter-clockwise
        # self.rotate(rate, dis_rot_distance, vel_left=0,
        #             vel_right=0.3, clockwise=False)
        # self.stop()

        # move forward
        # self.forward(rate, self.desired_distance, vel_left=0.4, vel_right=0.42)
        # self.stop(seconds=5)

        # TODO: clockwise circular movement

    # https://docs.duckietown.org/daffy/duckietown-robotics-development/out/new_duckiebot_functionality.html
    def on_shutdown(self):
        """Shutdown procedure.
        Publishes a zero velocity command at shutdown."""
        try:
            for i in range(15):
                self.stop()
        except:
            super(BasicMovemenNode, self).on_shutdown()


if __name__ == '__main__':
    node = BasicMovemenNode(node_name='basic_movement_node')
    node.run()

    # Plotting the rosbag data
    node.read_from_bag()

    #rospy.spin()
    node.bag.close()

    rospy.signal_shutdown('Duckiebot quacks goodbye!')