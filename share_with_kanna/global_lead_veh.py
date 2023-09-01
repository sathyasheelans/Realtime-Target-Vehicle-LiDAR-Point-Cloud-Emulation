#! /usr/bin/env python

import rospy
import numpy as np
import math
import os

from geometry_msgs.msg import TwistStamped, Pose, Vector3, Twist
from novatel_oem7_msgs.msg import INSPVA
from std_msgs.msg import Header, Float32MultiArray, Bool, Float64MultiArray, Float64
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import NavSatFix
from autoware_sim_op.msg import POV_OBJECTS, POV_OBJECT

from ss_core.message.map.smart_center import SmartCenter
from ss_core.message.map.mcity_highway import Mcity
from ss_core.utils.conversions import degree2radians, radians2degree, normalizeAngle, latlon2mgrs, mgrs2latlon, euler2quaternion

def wrap_to_pi(x):
    while x > np.pi:
        x -= 2*np.pi 
    while x < -np.pi:
        x += 2*np.pi
    return x
    # return ((x + np.pi) % (2. * np.pi)) - np.pi

def wrap_to_2pi(x):
    while x > 2*np.pi:
        x -= 2*np.pi 
    while x < 0:
        x += 2*np.pi
    return x

def not_zero(x, eps=1e-2):
    if abs(x) > eps:
        return x
    elif x > 0:
        return eps
    else:
        return -eps

class Bicycle():
    name = "Bicycle"

    def __init__(self, lr=1.,lf=1.,min_velocity=0, max_velocity=100):
        self.lr = lr 
        self.lf = lf 
        self.min_velocity = min_velocity
        self.max_velocity = max_velocity

    def step(self,s0, u0, delta_t):
        a, wh = u0
        [ x, y, v, yaw] = s0
        x += v * math.cos(yaw) * delta_t
        y += v * math.sin(yaw) * delta_t
        yaw += v / (self.lr+self.lf) * math.tan(wh) * delta_t
        v += a * delta_t
        if v > self.max_velocity:
            v = self.max_velocity
        if v < self.min_velocity:
            v = self.min_velocity
        yaw = wrap_to_2pi(yaw)
        return np.array([x, y, v, yaw])

    # def step(self, s0, u0, delta_t):
    # 	[x0, y0, v0, h0] = s0

    # 	return np.array(self._update(s0, delta_t, u0[0], u0[1]))

class LaneChange:

    def __init__(self, veh_l=5, tau_a=0.6, tau_ds=0.2, max_steering=np.pi/9):
        self.KP_HEADING = 1/tau_ds
        self.KP_LATERAL = (1./3.)*self.KP_HEADING
        self.MAX_STEERING_ANGLE = max_steering
        self.veh_l = veh_l

    def get_steer(self, s0, lateral_target=None, heading_comp=0):
        lateral_offset = s0[1]-lateral_target
        if abs(lateral_offset) > 1.:
            lateral_offset = np.sign(lateral_offset)*1.
        lane_coords = [s0[0], lateral_offset]
        # Lateral position control
        lateral_speed_command = - self.KP_LATERAL * lane_coords[1]
        # Lateral speed to heading
        heading_command = np.arcsin(np.clip(lateral_speed_command / not_zero(s0[2]), -1, 1))
        heading_ref = heading_comp + np.clip(heading_command, -np.pi/4, np.pi/4)
        # Heading control
        heading_rate_command = self.KP_HEADING * wrap_to_pi(heading_ref - s0[3])
        # Heading rate to steering angle
        steering_angle = np.arcsin(np.clip(self.veh_l / 2 / not_zero(s0[2]) * heading_rate_command, -1, 1))
        steering_angle = np.clip(steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
        
        return steering_angle

class SmartCenterEnv:

    def __init__(self, lanechange, model,map ="SC", dt=0.1):
        self.lanechange, self.model = lanechange, model
        self.dt = dt 
        if map == "SC":
            self.map = SmartCenter(file_name=(os.path.expanduser('~')+"/standard_ws/src/as_aw_pkgs/aw_platform/map/lanelet2_map.osm"))
        elif map == "MCity":
            self.map = SmartCenter(file_name=(os.path.expanduser('~')+"/standard_ws/src/as_aw_pkgs/aw_platform/map/mcity_highway_lanelet2.osm"))

    def step_vehicle(self, state, overlap_off=0, acc=0, k=0, b=0, heading_diff=0, inverse = False):
        x, y, v, h = state
        if inverse:
            lateral_target = (-(k*x-y+b)-overlap_off)/np.sqrt(k**2+1)
        else:
            lateral_target = ((k*x-y+b)-overlap_off)/np.sqrt(k**2+1)
        # print(lateral_target)
        # wrap_to_2pi(h_cur)+state[3]
        steer = self.lanechange.get_steer([0,0,state[2],0], lateral_target=lateral_target, heading_comp=heading_diff-h)
        state_ = self.model.step([0, 0, v, 0], [acc, steer], self.dt)
        dx, dy, dv, dh = state_[0], state_[1], state_[2]-v, state_[3]
        dd = np.sqrt(dx**2 + dy**2)
        x_ = x - dd * np.cos(h-np.pi/2)
        y_ = y - dd * np.sin(h-np.pi/2)
        v_ = v + dv 
        h_ = dh + h 

        return np.array([x_, y_, v_, h_])

class GlobalLeadVehNode:
    def __init__(self):
        rospy.init_node("global_lead_veh_node")

        frequency = 10
        self.Rate = rospy.Rate(frequency)
        self.map = rospy.get_param('~map_init', "SC")
        if self.map == "SC":
            self.sc_obj = SmartCenter(file_name=(os.path.expanduser('~')+"/standard_ws/src/as_aw_pkgs/aw_platform/map/lanelet2_map.osm"))
        elif self.map == "MCity":
            self.sc_obj = Mcity(file_name=(os.path.expanduser('~')+"/standard_ws/src/as_aw_pkgs/aw_platform/map/mcity_highway_lanelet2.osm"))
        print(self.map)
        self.dx = rospy.get_param('init_lon_dist',20)
        self.dy = rospy.get_param('init_lat_dist', 0.0)
        self.v0 = rospy.get_param('init_sv_speed', 15)
        self.v1 = rospy.get_param('init_pov_speed', 20)
        self.dh = rospy.get_param('init_heading_diff', 0.001)
        self.pov_length = rospy.get_param('object_length', 3.0)
        self.pov_width = rospy.get_param('object_width', 2.0)
        self.pov_height = rospy.get_param('object_height', 1.5)
        self.speed_tracking = rospy.get_param('speed_tracking', False)
        self.acc = rospy.get_param('init_pov_acc', 0.0)
        self.overlap_offset = rospy.get_param("overlap_offset", 0.0)
        self.pov_droupout_prob = rospy.get_param('dropout_prob',0.0)
        self.obj_length = rospy.get_param('object_length', 3.0)
        self.obj_width = rospy.get_param('object_width', 2.0)
        self.obj_height = rospy.get_param('object_height', 1.5)

        self.s0 = [self.dx,self.dy,self.v0,self.v1,self.dh]
        self.num_vels = 30                  # 3 second window
        self.current_velocity_list = []
        self.vel_tol = 0.01
        self.dist_tol = 1.5
        self.time_tol = 0.2

        self.engage_data = None
        self.scenario_start_check = False
        self.scenario_valid = False

        # init lane detection
        self.k, self.b, self.h_cur = None, None, None

        self.sv_pos = None
        self.sv_ori = None
        self.sv_vel = None
        self.pov_state = None
        lc = LaneChange()
        bc = Bicycle()
        self.env = SmartCenterEnv(lc, bc)
        
        rospy.Subscriber("/novatel/oem7/inspva",INSPVA, self.gpsCallback)
        rospy.Subscriber("/current_velocity",TwistStamped,self.velocityCallback)
        rospy.Subscriber("/vehicle/engage",Bool, self.engageCallback)
        rospy.Subscriber("/scenario_started", Bool, self.scenarioStartCallback)

        self.global_lead_pub = rospy.Publisher('/global_pov_objects',POV_OBJECTS, queue_size=1)
        self.pov_dropout_pub = rospy.Publisher('/dropout_prob',Float32MultiArray, queue_size=1)
        self.scenario_valid_pub = rospy.Publisher('/scenario_valid', Bool, queue_size=1)
        print("start lead veh loop")
        self.loop()

    def loop(self):
        try:     
            scenario_initialized = False
            while not rospy.is_shutdown():
                if self.sv_pos is not None and self.sv_vel is not None and self.engage_data is not None:
                    if self.scenario_start_check and not scenario_initialized:
                            print("begin initializion of pov")
                            [self.pov_lat,self.pov_lon, self.pov_ele],self.pov_vel, self.pov_ori = \
                                    self.sc_obj.ask_Od_ros(self.s0, self.sv_pos, self.sv_vel, self.sv_ori, return_mgrs=False,last_point_return = False)
                            if self.pov_lon is None and self.pov_lat is None:
                                break
                            else:
                                [self.pov_roll,self.pov_pitch,self.pov_azimuth] = -radians2degree(normalizeAngle(self.pov_ori[0],min=0,max=2*np.pi,mode="rad")),-radians2degree(normalizeAngle(self.pov_ori[1],min=0,max=2*np.pi,mode="rad")),-radians2degree(normalizeAngle(self.pov_ori[2],min=0,max=2*np.pi,mode="rad"))
                                self.pov_azimuth = normalizeAngle(self.pov_azimuth,min=0,max=360,mode="deg")
                                self.pov_x, self.pov_y, self.pov_z = latlon2mgrs(self.pov_lat, self.pov_lon, self.pov_ele)
                                self.pov_v = self.v1 
                                self.pov_h = degree2radians(self.pov_azimuth)
                                self.state = [self.pov_x, self.pov_y, self.pov_v, self.pov_h]

                                sv_x, sv_y , _ = latlon2mgrs(self.sv_pos[0],self.sv_pos[1])
                                sv_id = self.env.map.get_laneid_from_point([sv_x, sv_y], input_xy=True)
                                pov_id = self.env.map.get_laneid_from_point([self.pov_x, self.pov_y], input_xy=True)                            
                                distance_diff = abs(self.s0[0]-np.sqrt((sv_x-self.pov_x)**2+(sv_y-self.pov_y)**2))
                                tol = abs(self.sv_vel[0]-self.s0[3])*self.time_tol + self.dist_tol
                                if (distance_diff <= tol) and (sv_id==pov_id):
                                    self.scenario_valid = True
                                scenario_initialized = True
                    if scenario_initialized:
                        print("lead POV initialized")
                        self.step_pov()
                        self.publish_lead_veh()
                        self.publish_pov_dropout()
                self.Rate.sleep()
        except KeyboardInterrupt:
            pass

    def step_pov(self):
        if self.k is None:
            lane_cur, lane_left, lane_right = self.env.map.segment_query(self.state)
            self.k, self.b, self.h_cur, self.inverse_sign = lane_cur
            self.h_cur = -self.h_cur
            self.h_cur = normalizeAngle(self.h_cur,min=0,max=2*np.pi,mode="rad")
            # h_check = self.state[-1]
            # print("h check::::::::::::::::::::::::::::::::::::",h_check)
            # if h_check>0:
            #     print("not negated")
            #     self.k = self.k
            #     self.b = self.b            
        heading_diff = self.h_cur
        pov_next = self.env.step_vehicle(self.state, overlap_off=self.overlap_offset, acc=self.acc, k=self.k, b=self.b, heading_diff=heading_diff, inverse = self.inverse_sign)
        [self.pov_lat, self.pov_lon, _] = mgrs2latlon(pov_next[0], pov_next[1])
        output = self.sc_obj.get_nearest_center_point_from_point([self.pov_lat, self.pov_lon])
        self.ele, self.pov_roll, self.pov_pitch, self.pov_azimuth = \
                output[2],\
                -radians2degree(output[3]),\
                -radians2degree(output[4]),\
                radians2degree(np.pi/2+pov_next[-1])
        self.pov_v = pov_next[2]
        self.state = pov_next

    def publish_lead_veh(self):  
        pov_object = POV_OBJECT()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        pov_object.header = header

        pov_object.scale_x = self.pov_length
        pov_object.scale_y = self.pov_width
        pov_object.scale_z = self.pov_height

        pov_pos = Pose()
        self.pov_x, self.pov_y, self.pov_z = latlon2mgrs(self.pov_lat, self.pov_lon, self.pov_ele)
        pov_pos.position.x = self.pov_x 
        pov_pos.position.y = self.pov_y 
        pov_pos.position.z = self.pov_z 
        pov_pos.orientation.x, pov_pos.orientation.y, pov_pos.orientation.z, pov_pos.orientation.w = \
            euler2quaternion(degree2radians(self.pov_roll), degree2radians(self.pov_pitch), degree2radians(self.pov_azimuth)) 
        pov_object.pose = pov_pos

        pov_object.speed = self.pov_v
        
        pov_pos_latlon = NavSatFix()
        pov_pos_latlon.header = header
        pov_pos_latlon.latitude = self.pov_lat
        pov_pos_latlon.longitude = self.pov_lon
        pov_pos_latlon.altitude = self.pov_ele
        pov_object.coordinate = pov_pos_latlon

        pov_objects = POV_OBJECTS()
        pov_objects.objects.append(pov_object)
        self.global_lead_pub.publish(pov_objects)
        self.scenario_valid_pub.publish(self.scenario_valid)

    def publish_pov_dropout(self):
        pov_droupout_prob_array = [self.pov_droupout_prob]
        self.pov_dropout_pub.publish(Float32MultiArray(data=pov_droupout_prob_array))

    def gpsCallback (self,msg):
        self.sv_pos = [msg.latitude, msg.longitude, msg.height]
        [sv_roll,sv_pitch,sv_azimuth] = [msg.roll, msg.pitch, msg.azimuth]
        self.sv_ori = (degree2radians(sv_roll),degree2radians(sv_pitch),degree2radians(sv_azimuth))


    def velocityCallback (self,msg):   
        linear_x_vel = msg.twist.linear.x
        linear_y_vel = msg.twist.linear.y
        linear_z_vel = msg.twist.linear.z
        self.sv_vel = [linear_x_vel,linear_y_vel,linear_z_vel]

    def engageCallback (self,msg):
        self.engage_data = msg.data

    def scenarioStartCallback (self,msg):
        self.scenario_start_check = msg.data

if __name__ == '__main__':
    GlobalLeadVehNode()

