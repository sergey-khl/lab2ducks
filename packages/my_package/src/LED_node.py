#!/usr/bin/env python3

import rospy

from duckietown_msg.srv import SetCustomLEDPattern, ChangePattern
from duckietown_msgs.srv import SetCustomLEDPatternResponse, ChangePatternResponse
from duckietown.msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA 

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType


class LEDNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LEDNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)

        self.LEDspattern = [[0.0,0.0,0.0]] * 5 

        self.veh_name = rospy.get_namespace().strip("/")

        self._LED_protocol = rospy.get_param("~LED_protocol")

        self.pattern = [[0,0,0]] * 5
        self.current_pattern_name = "LIGHT_OFF"
        self.changePattern(self.current_pattern_name)

        # -- Publishers -- 
        self.pub_leds = rospy.Publisher(
            "~led_pattern", LEDPattern, queue_size=1, dt_topic_type=TopicType.DRIVER
        )

        # -- Service -- 

        self.srv_set_pattern = rospy.Service(
            "~set_pattern", ChangePattern, self.srvSetPattern
        )


        self.changePattern("WHITE")
        self.log("lights activated!")


    def publishLEDs(self):
        LEDPattern_msg = LEDPattern()
        for i in range(5):
            rgba = ColorRGBA()
            rgba.r = self.LEDspattern[i][0]
            rgba.g = self.LEDspattern[i][1]
            rgba.b = self.LEDspattern[i][2]
            rgba.a = 1.0
            LEDPattern_msg.rgb_vals.append(rgba)
        self.pub_leds.publish(LEDPattern_msg)


    def changePattern(self, pattern_name):
        if pattern_name:
            if self.current_pattern_name == pattern_name and pattern_name != "custom":
                return 
            elif pattern_name.strip("'").strip('"') in self._LED_protocol["signals"]:
                self.current_pattern_name = pattern_name

            color_list = self._LED_protocol["signals"][pattern_name]["color_list"]

            if type(color_list) is str:
                self.pattern = [self._LED_protocol["colors"][color_list]]*5

            self.frequency_mask = self._LED_protocol["signals"][pattern_name]["frequency_mask"]
            self.frequency = self._LED_protocol["signals"][pattern_name]["frequency"]

            if self.frequency == 0:
                self.updateLEDs()

            self.log("Pattern changed to (%r), cycles: %s " % (pattern_name, self.frequency))

    