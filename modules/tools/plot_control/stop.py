#!/usr/bin/env python
import sys

import gflags
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
from std_msgs.msg import String

from modules.control.proto import control_cmd_pb2

def talker():
    pub = rospy.Publisher('/apollo/control', control_cmd_pb2.ControlCommand, queue_size=10)
    rospy.init_node('control-publisher-node', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    i = 0
    while not rospy.is_shutdown():
        control_cmd_pb = control_cmd_pb2.ControlCommand()
        # filling header
        # control_cmd_pb.header.timestamp_sec = 1553603065.74
        control_cmd_pb.header.module_name = "control"
        control_cmd_pb.header.sequence_num = i
        i=i+1
        control_cmd_pb.header.status.error_code = 0

        control_cmd_pb.steering_target = 0.0
        control_cmd_pb.steering_rate = 0.0
        control_cmd_pb.throttle = 0.0
        control_cmd_pb.brake = 50.0
        control_cmd_pb.engage_advice.advice = 2
        control_cmd_pb.gear_location = 1

        rospy.loginfo(control_cmd_pb)
        pub.publish(control_cmd_pb)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass