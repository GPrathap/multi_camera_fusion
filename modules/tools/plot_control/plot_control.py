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

BRAKE_LINE_DATA = []
TROTTLE_LINE_DATA = []
STEERING_LINE_DATA = []

TOTAL_TIME_DATA = []

STEERING_LAT_DATA = []
STEERING_HEADING_DATA = []
STEERING_FEEDBACK_DATA = []
STEERING_FEEDFORWARD_DATA = []
STEERING_POSITION_DATA = []

LAT_ERR_DATA = []
HEAD_ERR_DATA = []

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Control plot data length")


def callback(data):
    global STEERING_LINE_DATA
    global TROTTLE_LINE_DATA, BRAKE_LINE_DATA, TOTAL_TIME_DATA
    global STEERING_LAT_DATA, STEERING_HEADING_DATA,STEERING_FEEDBACK_DATA, STEERING_FEEDFORWARD_DATA, STEERING_POSITION_DATA
    global LAT_ERR_DATA, HEAD_ERR_DATA

    control_cmd_pb = control_cmd_pb2.ControlCommand()
    control_cmd_pb.CopyFrom(data)

    STEERING_LINE_DATA.append(control_cmd_pb.steering_target)
    if len(STEERING_LINE_DATA) > FLAGS.data_length:
        STEERING_LINE_DATA = STEERING_LINE_DATA[-FLAGS.data_length:]

    BRAKE_LINE_DATA.append(control_cmd_pb.brake)
    if len(BRAKE_LINE_DATA) > FLAGS.data_length:
        BRAKE_LINE_DATA = BRAKE_LINE_DATA[-FLAGS.data_length:]

    TROTTLE_LINE_DATA.append(control_cmd_pb.throttle)
    if len(TROTTLE_LINE_DATA) > FLAGS.data_length:
        TROTTLE_LINE_DATA = TROTTLE_LINE_DATA[-FLAGS.data_length:]

    TOTAL_TIME_DATA.append(control_cmd_pb.latency_stats.total_time_ms)
    if len(TOTAL_TIME_DATA) > FLAGS.data_length:
        TOTAL_TIME_DATA = TOTAL_TIME_DATA[-FLAGS.data_length:]

    STEERING_LAT_DATA.append(control_cmd_pb.debug.simple_lat_debug.steer_angle_lateral_contribution)
    if len(STEERING_LAT_DATA) > FLAGS.data_length:
        STEERING_LAT_DATA = STEERING_LAT_DATA[-FLAGS.data_length:]

    STEERING_HEADING_DATA.append(control_cmd_pb.debug.simple_lat_debug.steer_angle_heading_contribution)
    if len(STEERING_HEADING_DATA) > FLAGS.data_length:
        STEERING_HEADING_DATA = STEERING_HEADING_DATA[-FLAGS.data_length:]

    STEERING_FEEDBACK_DATA.append(control_cmd_pb.debug.simple_lat_debug.steer_angle_feedback)
    if len(STEERING_FEEDBACK_DATA) > FLAGS.data_length:
        STEERING_FEEDBACK_DATA = STEERING_FEEDBACK_DATA[-FLAGS.data_length:]

    STEERING_FEEDFORWARD_DATA.append(control_cmd_pb.debug.simple_lat_debug.steer_angle_feedforward)
    if len(STEERING_FEEDFORWARD_DATA) > FLAGS.data_length:
        STEERING_FEEDFORWARD_DATA = STEERING_FEEDFORWARD_DATA[-FLAGS.data_length:]

    STEERING_POSITION_DATA.append(control_cmd_pb.debug.simple_lat_debug.steering_position)
    if len(STEERING_POSITION_DATA) > FLAGS.data_length:
        STEERING_POSITION_DATA = STEERING_POSITION_DATA[-FLAGS.data_length:]

    LAT_ERR_DATA.append(control_cmd_pb.debug.simple_lat_debug.lateral_error)
    if len(LAT_ERR_DATA) > FLAGS.data_length:
        LAT_ERR_DATA = LAT_ERR_DATA[-FLAGS.data_length:]

    HEAD_ERR_DATA.append(control_cmd_pb.debug.simple_lat_debug.heading_error)
    if len(HEAD_ERR_DATA) > FLAGS.data_length:
        HEAD_ERR_DATA = HEAD_ERR_DATA[-FLAGS.data_length:]


def listener():
    rospy.init_node('control_listener', anonymous=True)
    rospy.Subscriber('/apollo/control', control_cmd_pb2.ControlCommand,
                     callback)

    # rospy.spin()


def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):
    brake_data = compensate(BRAKE_LINE_DATA)
    brake_line.set_ydata(brake_data)

    throttle_data = compensate(TROTTLE_LINE_DATA)
    throttle_line.set_ydata(throttle_data)

    steering_data = compensate(STEERING_LINE_DATA)
    steering_line.set_ydata(steering_data)

    total_time_data = compensate(TOTAL_TIME_DATA)
    #total_time_line.set_ydata(total_time_data)

    steering_lat_data = compensate(STEERING_LAT_DATA)
    steer_angle_lateral_line.set_ydata(steering_lat_data)

    steering_heading_data = compensate(STEERING_HEADING_DATA)
    steer_angle_heading_line.set_ydata(steering_heading_data)

    steering_feedback_data = compensate(STEERING_FEEDBACK_DATA)
    steer_angle_feedback_line.set_ydata(steering_feedback_data)

    steering_feedforward_data = compensate(STEERING_FEEDFORWARD_DATA)
    steer_angle_feedforward_line.set_ydata(steering_feedforward_data)

    steering_position_data = compensate(STEERING_POSITION_DATA)
    steering_position_line.set_ydata(steering_position_data)

    lat_err_data = compensate(LAT_ERR_DATA)
    lat_err_line.set_ydata(lat_err_data)

    head_err_data = compensate(HEAD_ERR_DATA)
    head_err_line.set_ydata(head_err_data)

    brake_text.set_text('brake = %.1f' % brake_data[-1])
    throttle_text.set_text('throttle = %.1f' % throttle_data[-1])
    steering_text.set_text('steering = %.1f' % steering_data[-1])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    listener()
    #fig, ax = plt.subplots()
    fig = plt.figure()
    ax = fig.add_subplot(311)
    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()
    steering_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'b', lw=3, alpha=0.5, label='steering')
    throttle_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'g', lw=3, alpha=0.5, label='throttle')
    brake_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'r', lw=3, alpha=0.5, label='brake')
    brake_text = ax.text(0.75, 0.85, '', transform=ax.transAxes)
    throttle_text = ax.text(0.75, 0.90, '', transform=ax.transAxes)
    steering_text = ax.text(0.75, 0.95, '', transform=ax.transAxes)
    ax.set_ylim(-100, 120)
    ax.set_xlim(-1 * FLAGS.data_length, 10)
    ax.legend(loc="upper left")

    #Timing
    # ax1 = fig.add_subplot(312)
    # total_time_line, = ax1.plot(
    #     Xs, [0] * FLAGS.data_length, 'b', lw=3, alpha=0.5, label='Total time, ms')
    # ax1.set_ylim(0, 50)
    # ax1.set_xlim(-1 * FLAGS.data_length, 10)
    # ax1.legend(loc="upper left")

    #Errors
    ax1 = fig.add_subplot(312)
    lat_err_line, = ax1.plot(
        Xs, [0] * FLAGS.data_length, 'b', lw=3, alpha=0.5, label='Lateral error, m')
    head_err_line, = ax1.plot(
        Xs, [0] * FLAGS.data_length, 'r', lw=3, alpha=0.5, label='Heading error, rad')
    ax1.set_ylim(-5, 5)
    ax1.set_xlim(-1 * FLAGS.data_length, 10)
    ax1.legend(loc="upper left")

    #Lateral degub
    ax2 = fig.add_subplot(313)
    steer_angle_lateral_line, = ax2.plot(
        Xs, [0] * FLAGS.data_length, 'b', lw=3, alpha=0.5, label='steer_angle_lateral')
    steer_angle_heading_line, = ax2.plot(
        Xs, [0] * FLAGS.data_length, 'g', lw=3, alpha=0.5, label='steer_angle_heading')
    steer_angle_feedback_line, = ax2.plot(
        Xs, [0] * FLAGS.data_length, 'r', lw=3, alpha=0.5, label='steer_angle_feedback')
    steer_angle_feedforward_line, = ax2.plot(
        Xs, [0] * FLAGS.data_length, 'c', lw=3, alpha=0.5, label='steer_angle_feedforward')
    steering_position_line, = ax2.plot(
        Xs, [0] * FLAGS.data_length, 'm', lw=3, alpha=0.5, label='steering_position')        
    ax2.set_ylim(-100, 120)
    ax2.set_xlim(-1 * FLAGS.data_length, 10)
    ax2.legend(loc="upper left")

    ani = animation.FuncAnimation(fig, update, interval=100)


    plt.show()
