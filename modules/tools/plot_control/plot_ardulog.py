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

from modules.control.proto import control_cmd_pb2
from ardulog.msg import Ardulog

SETPOINT_DATA = []
ANGLE_DATA = []
CONTROL_DATA = []

P_DATA = []
I_DATA = []
D_DATA = []

STEERING_POSITION_DATA = []

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Control plot data length")


def callback(ardulog_msg):
    global SETPOINT_DATA, ANGLE_DATA, CONTROL_DATA, P_DATA, I_DATA, D_DATA, STEERING_POSITION_DATA

    #ardulog_msg = Ardulog()
    #ardulog_msg.CopyFrom(data)

    SETPOINT_DATA.append(ardulog_msg.setpoint)
    if len(SETPOINT_DATA) > FLAGS.data_length:
        SETPOINT_DATA = SETPOINT_DATA[-FLAGS.data_length:]
        
    ANGLE_DATA.append(ardulog_msg.angle)
    if len(ANGLE_DATA) > FLAGS.data_length:
        ANGLE_DATA = ANGLE_DATA[-FLAGS.data_length:]

    CONTROL_DATA.append(ardulog_msg.control)
    if len(CONTROL_DATA) > FLAGS.data_length:
        CONTROL_DATA = CONTROL_DATA[-FLAGS.data_length:]

    P_DATA.append(ardulog_msg.p)
    if len(P_DATA) > FLAGS.data_length:
        P_DATA = P_DATA[-FLAGS.data_length:]
    
    I_DATA.append(ardulog_msg.i)
    if len(I_DATA) > FLAGS.data_length:
        I_DATA = I_DATA[-FLAGS.data_length:]

    D_DATA.append(ardulog_msg.d)
    if len(D_DATA) > FLAGS.data_length:
        D_DATA = D_DATA[-FLAGS.data_length:]




def listener():
    rospy.init_node('ardulog_listener', anonymous=True)
    rospy.Subscriber('/ardulog', Ardulog, callback)

    # rospy.spin()


def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):

    setpoint_data = compensate(SETPOINT_DATA)
    setpoint_line.set_ydata(setpoint_data)

    angle_data = compensate(ANGLE_DATA)
    angle_line.set_ydata(angle_data)

    control_data = compensate(CONTROL_DATA)
    control_line.set_ydata(control_data)

    p_data = compensate(P_DATA)
    p_line.set_ydata(p_data)

    i_data = compensate(I_DATA)
    i_line.set_ydata(i_data)

    d_data = compensate(D_DATA)
    d_line.set_ydata(d_data)

    setpoint_text.set_text('target steering = %.1f' % setpoint_data[-1])
    angle_text.set_text('cur steering = %.1f' % angle_data[-1])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    listener()
    #fig, ax = plt.subplots()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    X = range(FLAGS.data_length)
    Xs = [i * -1.0/60.0 for i in X]
    Xs.sort()
    setpoint_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'b', lw=3, alpha=0.5, label='setpoint')
    angle_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'm', lw=3, alpha=0.5, label='angle')
    control_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'r', lw=3, alpha=0.5, label='control')
    p_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'g', lw=3, alpha=0.5, label='P component')   
    i_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'y', lw=3, alpha=0.5, label='I component')   
    d_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'c', lw=3, alpha=0.5, label='D component')  

    setpoint_text = ax.text(0.75, 0.95, '', transform=ax.transAxes)
    angle_text = ax.text(0.75, 0.1, '', transform=ax.transAxes)
    ax.set_ylim(-100, 120)
    ax.set_xlim(-1.0/60.0 * FLAGS.data_length, 0)
    ax.legend(loc="upper left")

    ani = animation.FuncAnimation(fig, update, interval=100)


    plt.show()
