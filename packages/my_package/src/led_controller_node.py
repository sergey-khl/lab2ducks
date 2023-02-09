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

class LEDControlNode(DTROS):
    """
    Switches the led color?
    """
    def __init__(self, node_name):
        super(LEDControlNode, self).__init__(
            node_name=node_name, node_type=NodeType.CONTROL
        )

        led = f'/{self.veh_name}/led_emitter_node/set_pattern'

        rospy.wait_for_service(led)

        try:
            change_led = rospy.ServiceProxy('GREEN', ChangePattern)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            rospy.loginfo("No light :(")
        


if __name__ == '__main__':
    node = LEDControlNode(node_name='led_controller_node')

    rospy.loginfo("Let there be light")


    rospy.spin()