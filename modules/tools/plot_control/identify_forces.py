#!/usr/bin/env python
import sys

import gflags
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
from std_msgs.msg import String

from modules.control.proto import control_cmd_pb2
from modules.canbus.proto import chassis_pb2

speed = 0

def chassis_callback(data):
    global speed
    chassis_pb = chassis_pb2.Chassis()
    chassis_pb.CopyFrom(data)
    speed = chassis_pb.speed_mps * 3.6

def talker():
    global speed
    rospy.init_node('control-publisher-node', anonymous=True)
    pub = rospy.Publisher('/apollo/control', control_cmd_pb2.ControlCommand, queue_size=10)
    rospy.Subscriber('/apollo/canbus/chassis', chassis_pb2.Chassis,
                        chassis_callback)
    rate = rospy.Rate(50) # 10hz
    i = 0
    accelerated = False
    while not rospy.is_shutdown():
        print("Speed = ", speed)
        control_cmd_pb = control_cmd_pb2.ControlCommand()
        # filling header
        # control_cmd_pb.header.timestamp_sec = 1553603065.74
        control_cmd_pb.header.module_name = "control"
        control_cmd_pb.header.sequence_num = i
        i=i+1
        control_cmd_pb.header.status.error_code = 0

        brake = 0.0
        throttle = 60.0

        if (speed>=40.0):
            accelerated = True
        if (accelerated):
            brake = 75.0
            throttle = 0.0
        
        control_cmd_pb.steering_target = 0.0
        control_cmd_pb.steering_rate = 0.0
        control_cmd_pb.throttle = throttle
        control_cmd_pb.brake = brake
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