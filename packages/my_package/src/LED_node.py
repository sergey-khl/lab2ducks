#!/usr/bin/env python3

import numpy as np
import os
import rospy

from duckietown_msg.srv import SetCustomLEDPattern, ChangePattern
from duckietown_msgs.srv import SetCustomLEDPatternResponse, ChangePatternResponse
from duckietown.msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA 

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType



class LEDNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(LEDNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)
        self.veh_name = rospy.get_namespace().strip("/")


        # Publishers
        self.pub_led = rospy.Publisher(
            "~led_pattern", LEDPattern, queue_size=1, dt_topic_type=TopicType.DRIVER
        )

        # Services 
        self.srv_set_LED_ = rospy.Service(
            "~set_custom_pattern", SetCustomLEDPattern
        )