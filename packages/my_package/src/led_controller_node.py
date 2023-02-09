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

        rospy.loginfo("Started led_control_service")
        

    def switch_led_colors(self, srv: ChangePattern):
        rospy.loginfo(srv.r)
        msg = create_led_msg([srv.r, srv.g, srv.b, srv.a])
        self.pub.publish(msg)
        return 1


if __name__ == '__main__':
    node = LEDControlNode(node_name='led_controller_node')
    rospy.loginfo("Starting led controller")

    # #rospy.init_node('led_controls_server')
    # def turn_off_leds():
    #     led_msg = create_led_msg([0.0, 0.0, 0.0])
    #     node.pub.publish(led_msg)

    # rospy.on_shutdown(turn_off_leds)

    # rospy.wait_for_service('led_control_service')

    # switch_led = rospy.ServiceProxy('led_control_service', ChangePattern)
    # resp1 = switch_led(0.0, 1.0, 1.0, 1.0)
    # rospy.loginfo(f"Got response: {resp1}")

    # rate = rospy.Rate(1)

    # while not rospy.is_shutdown():
    #     for rgb in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
    #                 [1.0, 1.0, 0.0], [0.0, 1.0, 1.0]]:
    #         r = switch_led(rgb[0], rgb[1], rgb[2], 1.0)
    #         rospy.loginfo(f"LEDs to color {rgb} with response {r}")
    #         rate.sleep()

    #rospy.signal_shutdown("Required")

    rospy.spin()