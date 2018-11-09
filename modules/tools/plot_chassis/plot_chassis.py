#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import sys

import gflags
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
from std_msgs.msg import String

from modules.canbus.proto import chassis_pb2

STEERING_LINE_DATA = []
VEL_LINE_DATA = []

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Control plot data length")


def callback(data):
    global STEERING_LINE_DATA, VEL_LINE_DATA


    chassis_pb = chassis_pb2.Chassis()
    chassis_pb.CopyFrom(data)

    STEERING_LINE_DATA.append(chassis_pb.steering_percentage)
    if len(STEERING_LINE_DATA) > FLAGS.data_length:
        STEERING_LINE_DATA = STEERING_LINE_DATA[-FLAGS.data_length:]

    VEL_LINE_DATA.append(chassis_pb.speed_mps)
    if len(VEL_LINE_DATA) > FLAGS.data_length:
        VEL_LINE_DATA = VEL_LINE_DATA[-FLAGS.data_length:]



def listener():
    rospy.init_node('chassis_listener', anonymous=True)
    rospy.Subscriber('/apollo/canbus/chassis', chassis_pb2.Chassis,
                     callback)

    # rospy.spin()


def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):

    steering_data = compensate(STEERING_LINE_DATA)
    steering_line.set_ydata(steering_data)
    steering_text.set_text('Steering = %.2f' % steering_data[-1])

    vel_data = compensate(VEL_LINE_DATA)
    vel_line.set_ydata(vel_data)
    vel_text.set_text('velocity = %.5f' % vel_data[-1])



if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    listener()
    #fig, ax = plt.subplots()
    fig = plt.figure()
    ax = fig.add_subplot(211)
    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()
    steering_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'b', lw=3, alpha=0.5, label='Steering, %')
    steering_text = ax.text(0.75, 0.85, '', transform=ax.transAxes)
    ax.set_ylim(-120, 120)
    ax.set_xlim(-1 * FLAGS.data_length, 10)
    ax.legend(loc="upper left")

    #Velocity
    ax1 = fig.add_subplot(212)
    vel_line, = ax1.plot(
        Xs, [0] * FLAGS.data_length, 'b', lw=3, alpha=0.5, label='Velocity, m/s')
    vel_text = ax1.text(0.75, 0.85, '', transform=ax1.transAxes)
    ax1.set_ylim(-10, 10)
    ax1.set_xlim(-1 * FLAGS.data_length, 10)
    ax1.legend(loc="upper left")

    ani = animation.FuncAnimation(fig, update, interval=100)


    plt.show()
