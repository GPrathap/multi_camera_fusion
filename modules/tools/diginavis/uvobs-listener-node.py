import sys
sys.path.insert(0, "/home/tmp/ros/lib/python2.7/site-packages/")

print (sys.path)
import argparse
import os
import logging
from logger import Logger
import util
import debug_topo
import rospy
from scipy import interpolate
import json
import numpy as np
from numpy import linalg 
import ConfigParser

from modules.routing.proto import routing_pb2
from modules.canbus.proto import chassis_pb2
from modules.common.proto import drive_state_pb2
from modules.planning.proto import planning_pb2
from modules.control.proto import pad_msg_pb2

import common.kv_db as kv_db
import common.proto_utils as proto_utils

from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


APOLLO_ROOT = os.path.join(os.path.dirname(__file__), '../../../')


class UvobsListener():
    def __init__(self, config_file):
       
        self.firstvalid = False
        self.logger = Logger.get_logger(tag="UvobsListener")
        self.logger.info("Starting UvobsListener...")
        
        try:
            self.config = ConfigParser.ConfigParser()
            self.config.readfp(open(config_file, "r"))
        except:
            self.logger.error("Cannot find file: " + record_file)
            sys.exit(0)

        self.chassis = chassis_pb2.Chassis()
        self.padmsg = pad_msg_pb2.PadMessage()
        self.route = routing_pb2.RoutingResponse()
        self.planning = planning_pb2.ADCTrajectory()

        self.chassis_received = False
        self.terminating = False
        self.sequence_num = 0
        self.start = 0
        self.end = 0
        self.previos_route_distance = 0
        self.current_route_distance = 0
        self.automode = False
        self.mapdir = ""
        self.central_curves = {}
        self.STOP_REASON_DESTINATION = 2

        # subscribe to RouteResponse
        rospy.Subscriber('/apollo/routing_response', routing_pb2.RoutingResponse,
                            self.route_response_callback, queue_size=1)

        rospy.Subscriber('/apollo/canbus/chassis', chassis_pb2.Chassis
                                , self.callback_chassis, queue_size=1)

        rospy.Subscriber('/apollo/control/pad', pad_msg_pb2.PadMessage
            , self.padmsg_callback)

        rospy.Subscriber('/apollo/planning', planning_pb2.ADCTrajectory
            , self.planning_callback)

        routing_path_topic = self.config.get('Configuration', 'hd_map_position_topic')
        self.vehicle_status = rospy.Publisher(routing_path_topic, String, queue_size=20, latch=True)
        
        map_type = kv_db.KVDB.Get('apollo:dreamview:map')
        if map_type is None:
            self.logger.error('Map type is not defined')
            exit(0)

        new_mapdir = self.get_map_dir_from_name(map_type)
        self.logger.info(new_mapdir)

        if (new_mapdir!= self.mapdir):
            self.reload_map(new_mapdir)
            self.logger.info('Map loaded')
        else:
            self.logger.error('Map could not be loaded')
            exit(0)
    

    def reload_map(self, new_mapdir):
        self.mapdir = new_mapdir
        self.graph = util.get_topodata(self.mapdir)
        for nd in self.graph.node:
            self.central_curves[nd.lane_id] = nd.central_curve


    def get_central_curve_with_s_range(self, central_curve, start_s, end_s):
        """return central lane [[x1,y1],[x2,y2],...] array"""
        node_x = []
        node_y = []
        for curve in central_curve.segment:
            px, py = proto_utils.flatten(curve.line_segment.point, ['x', 'y'])
            node_x.extend(px)
            node_y.extend(py)
        start_plot_index = 0
        end_plot_index = len(node_x)
        node_s = debug_topo.calculate_s(node_x, node_y)
        for i in range(len(node_s)):
            if node_s[i] >= start_s:
                start_plot_index = i
                break
        for i in range(len(node_s) - 1, -1, -1):
            if node_s[i] <= end_s:
                end_plot_index = i + 1
                break
        # transform to point array
        ptx = []
        for i in range(start_plot_index, end_plot_index):
            ptx.append([node_x[i], node_y[i], 0.0])
        return ptx


    def route_response_callback(self, data):
        try:
            self.route.CopyFrom(data)            
            self.send_routing_request()
        except Exception:
            self.logger.error("Something went wrong went sending routing path...")
            # exit(0)
            pass
            
    def send_routing_request(self):
        lanes = []
        self.logger.info(self.route)
        for rd in self.route.road:
            for ps in rd.passage:
                for seg in ps.segment:
                    lanes.append (seg)

        routhing_paths = []
        for lane in lanes:
            lane_id = lane.id
            color = (0,100,0)
            points = self.get_central_curve_with_s_range(
                self.central_curves[lane_id],
                lane.start_s,
                lane.end_s)
            routhing_paths.append(points)

        center_y = 6181144.7;
        center_x = 358177.2;
        center_position = [center_x, center_y, 0.0] 
        routePath = []
        for path in routhing_paths:
            path_sector = {
                "sector" : "U39",
                "center" : center_position,
                "point" : path
            }
            self.current_route_distance += linalg.norm(np.array(path))
            routePath.append(path_sector)

        self.logger.info("Distance between current and next paths: " + str(self.current_route_distance) + ", " + str(self.previos_route_distance))
        if(np.abs(self.current_route_distance - self.previos_route_distance ) < 0.2):
            self.logger.warn("Routhing path will not be sent, assuming as the previous path")
        else:  
            time_now = rospy.get_time()
            response = {
                "type": "RoutePath",
                "routingTime": time_now,
                "projection": "+proj=utm +zone=39 +ellps=WGS84 +datum=WGS84 +units=m +no_defs",
                "routePath" : routePath
            } 
            response = json.dumps(response)
            # response = str(response)
            while self.vehicle_status.get_num_connections() == 0:
                self.logger.info("Waiting for at least one subscriber to connect")
                rospy.sleep(1)

            self.vehicle_status.publish(response)
            self.logger.info(response)
            self.start = 1
        
        self.previos_route_distance = self.current_route_distance
   

    def get_map_dir_from_name(self, map_name):
        return os.path.normpath(
            os.path.join(APOLLO_ROOT, 'modules/map/data/',map_name.replace(' ', '_').lower()))

    def callback_chassis(self, data):
            self.chassis.CopyFrom(data)
            self.automode = (self.chassis.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE)
            self.carspeed = self.chassis.speed_mps
            # self.logger.info(self.chassis)
            if((self.start == 1) and (self.chassis.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE)):
                self.logger.info(self.start)
                response = {
                    "type" :  "SendStartPosition",
                    "code" :  "COMPLETE_AUTO_DRIVE"
                }
                response = json.dumps(response)
                while self.vehicle_status.get_num_connections() == 0:
                    self.logger.info("Waiting for at least one subscriber to connect")
                    rospy.sleep(1)
                self.vehicle_status.publish(response)
                self.start = 0
        
    def padmsg_callback(self, data):
            self.padmsg.CopyFrom(data)
            self.logger.info("Calling padding message")
            self.logger.info(self.padmsg)

    def planning_callback(self, data):
        self.planning.CopyFrom(data)
        self.logger.info("Calling planning message")
        stop_code = self.planning.decision.main_decision.stop.reason_code
        stop_point = self.planning.decision.main_decision.stop.stop_point
        self.logger.info(self.planning.decision.main_decision)
        self.logger.info(self.carspeed)
        
        if((stop_code == self.STOP_REASON_DESTINATION) and (self.carspeed < 0.1)):
            response = {
                "type" :  "StopDecision",
                "reason" : "STOP_REASON_DESTINATION",
                "code" : 2
            }
            while self.vehicle_status.get_num_connections() == 0:
                self.logger.info("Waiting for at least one subscriber to connect")
                rospy.sleep(1)
            response = json.dumps(response)
            # response = str(response)
            self.vehicle_status.publish(response)
       
        

def main():
   
    rospy.init_node('uvobs_listener_node', anonymous=True)
    Logger.config(
        log_file=os.path.join(APOLLO_ROOT, 'data/log/uvobs_listener.log'),
        use_stdout=True,
        log_level=logging.INFO)

    config_file = os.path.join(APOLLO_ROOT, 'modules/tools/diginavis/conf/uvobs_config.conf')
    uvobs_listener = UvobsListener(config_file)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    

if __name__ == '__main__':
    main()
