#!/usr/bin/env python3
import rospy
import os
import numpy as np
import rosbag
import rospy
import time

from duckietown_msgs.srv import ChangePattern, SetCustomPattern, ChangePatternResponse

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

hostname = os.environ['VEHICLE_NAME']


class LEDControlNode(DTROS):
    """
    Switches the led color?
    """
    def __init__(self, node_name):
        super(LEDControlNode, self).__init__(
            node_name=node_name, node_type=NodeType.CONTROL
        )

        self.veh_name = rospy.get_namespace().strip("/")

        led = f'/{self.veh_name}/led_emitter_node/set_custom__pattern'

        # rospy.wait_for_service(led)

        change_led = rospy.ServiceProxy(led, SetCustomPattern)

        msg = LEDPattern()
        msg.color_list = ['green', 'green', 'green', 'green', 'green']
        msg.color_mask = [1,1,1,1,1]
        msg.frequency = 0
        msg.frequency_mask = [0,0,0,0,0]

        change_led(change_led)


        self.log(str('hello'))
        

if __name__ == '__main__':
    node = LEDControlNode(node_name='led_controller_node')

    rospy.spin()