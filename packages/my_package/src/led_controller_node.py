#!/usr/bin/env python3
import rospy
import os
import numpy as np
import rosbag
import rospy
import time

from duckietown_msgs.srv import ChangePattern, ChangePatternResponse

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

hostname = os.environ['VEHICLE_NAME']


def create_led_msg(colors):
    """ Creates an led message with the colors set to values from a tuple

    Args:
        Colors (list[float]): A list of 3 floats in the order rgb
    """
    led_msg = LEDPattern()

    for i in range(5):
        rgba = ColorRGBA()
        rgba.r = colors[0]
        rgba.g = colors[1]
        rgba.b = colors[2]
        rgba.a = 1.0
        led_msg.rgb_vals.append(rgba)

    return led_msg



class LEDControlNode(DTROS):
    """
    Switches the led color?
    """
    def __init__(self, node_name):
        super(LEDControlNode, self).__init__(
            node_name=node_name, node_type=NodeType.CONTROL
        )

        self.LEDspattern = [[0.0, 0.0, 0.0]] * 5


        self.pub = rospy.Publisher(
            f'/{hostname}/led_emitter_node/led_pattern',
            LEDPattern,
            queue_size=10,
        )
        
        self.serv = rospy.Service(
            f'/{hostname}/led_emitter_node/set_pattern', 
            ChangePattern, 
            self.switch_led_colors
        )


        self.LEDspattern = [[1, 1, 1]] * 5
        self.changePattern("WHITE")

        rospy.loginfo("Started led_control_service")


if __name__ == '__main__':
    node = LEDControlNode(node_name='led_controller_node')
    rospy.loginfo("Starting led controller")

    rospy.spin()