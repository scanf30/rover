#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from fruit_tracker.cfg import HSVConfig
	
def callback(config, level):
    
    return config

if __name__ == "__main__":

    rospy.init_node("fruit_tracker_server", anonymous = False)

    srv = Server(HSVConfig, callback)
    rospy.spin()
