#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import time
import subprocess
import std_srvs.srv
import tf
import actionlib
import cv2
import math
import numpy as np
from numpy.random import randn
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Quaternion,PoseWithCovarianceStamped,PoseArray,Pose
from sensor_msgs.msg import LaserScan,Imu
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from nav_msgs.msg import OccupancyGrid

PARTICLE_NUM = 100

class Localization:
    def __init__(self):
        #publisher & subscriber
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
        self.particle_pub = rospy.Publisher('/e_particles',PoseArray,queue_size=1)
        self.localized_pose_pub = rospy.Publisher('/localized_robot_pose',Marker,queue_size=1)
        self.simulated_pose_pub = rospy.Publisher('/simulated_robot_pose',Marker,queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.LaserCB)
        self.robot_pose_sub= rospy.Subscriber('/gazebo/model_states',ModelStates,self.visualizeSimulatedRobotPoseCB)
        self.map_sub = rospy.Subscriber('/map',OccupancyGrid,self.mapCB)
        self.imu_sub = rospy.Subscriber('/imu',Imu,self.BaseCB)
        #particles
        self.particles = []
        self.initParticle()
        #map
        self.mymap = []
        #for odometry
        self.vel_x = 0.0
        self.dist_x = 0.0
        self.angle_z = 0.0
        self.time = time.time()
        #lidar
        #self.lidar_dist = np.array()

#---------------------------CallBack
    def BaseCB(self,msg):
        time_diff = time.time() - self.time
        self.time = time.time()

        self.vel_x += msg.linear_acceleration.x * time_diff
        self.dist_x += self.vel_x * time_diff
        self.angle_z += msg.angular_velocity.z * time_diff
        #print self.vel_x, self.vel_y, self.dist_x, self.dist_y
        print msg.linear_acceleration.x

    def LaserCB(self,laser_scan):
        self.lidar_dist = np.array(laser_scan.ranges)
        #print self.lidar_dist.shape

    def mapCB(self,msg):
        PIXEL_UNIT = 20#２０ピクセル１メートル
        width = int(msg.info.width)
        height = int(msg.info.height)
        mymap = np.array(msg.data)
        mymap = np.reshape(mymap,(width,height))
        #print msg.info.origin

    def visualizeSimulatedRobotPoseCB(self,model_state):
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "simulated_robot_pose"
        marker_data.id = 1
        marker_data.action = Marker.ADD
        marker_data.pose.position.x = model_state.pose[2].position.x
        marker_data.pose.position.y = model_state.pose[2].position.y
        marker_data.pose.position.z = 0
        marker_data.pose.orientation.x = model_state.pose[2].orientation.x
        marker_data.pose.orientation.y = model_state.pose[2].orientation.y
        marker_data.pose.orientation.z = model_state.pose[2].orientation.z
        marker_data.pose.orientation.w = model_state.pose[2].orientation.w
        marker_data.color.r = 0.0
        marker_data.color.g = 0.0
        marker_data.color.b = 1.0
        marker_data.color.a = 1.0
        marker_data.scale.x = 0.3
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.1
        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0
        self.simulated_pose_pub.publish(marker_data)

#--------------------------
    def initParticle(self):
        rn = randn(2*PARTICLE_NUM)
        for i in range(0,PARTICLE_NUM):
            self.particles.append({"x":rn[i*2],"y":rn[i*2+1],"pitch":0.5*math.pi})
    
    def visualizeParticle(self):
        particle_msg = PoseArray()
        particle_msg.header.frame_id = "map"
        particle_msg.header.stamp = rospy.Time.now()
        for i in range(0,PARTICLE_NUM):
            pose = Pose()
            pose.position.x = self.particles[i]["x"]
            pose.position.y = self.particles[i]["y"]
            pose.position.z = 0
            q = tf.transformations.quaternion_from_euler(0,0,self.particles[i]["pitch"])
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            particle_msg.poses.append(pose)
        self.particle_pub.publish(particle_msg)

    def visualizeLocalizedRobotPose(self):
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "localized_robot_pose"
        marker_data.id = 1
        marker_data.action = Marker.ADD
        x = 0.0
        y = 0.0
        pitch = 0.0
        for i in range(0,PARTICLE_NUM):
            x += self.particles[i]["x"]
            y += self.particles[i]["y"]
            pitch += self.particles[i]["pitch"]
        x = x/PARTICLE_NUM
        y = y/PARTICLE_NUM
        pitch = pitch/PARTICLE_NUM
        q = tf.transformations.quaternion_from_euler(0,0,pitch)
        marker_data.pose.position.x = x
        marker_data.pose.position.y = y
        marker_data.pose.position.z = 0
        marker_data.pose.orientation.x = q[0]
        marker_data.pose.orientation.y = q[1]
        marker_data.pose.orientation.z = q[2]
        marker_data.pose.orientation.w = q[3]
        marker_data.color.r = 0.0
        marker_data.color.g = 1.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0
        marker_data.scale.x = 0.3
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.1
        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0
        self.localized_pose_pub.publish(marker_data)

        
    def updateParticle(self):
        rn = randn(2*PARTICLE_NUM)
        for i in range(0,PARTICLE_NUM):
            self.particles[i]["x"] += self.dist_x * math.cos(self.particles[i]["pitch"]) + rn[i*2]   * 0.1
            self.particles[i]["y"] -= self.dist_x * math.sin(self.particles[i]["pitch"]) + rn[i*2+1] * 0.1
            self.particles[i]["pitch"] += self.angle_z
        self.dist_x = 0.0
        self.angle_z = 0.0

    def Localization(self):
        self.visualizeParticle()
        self.visualizeLocalizedRobotPose()
        self.updateParticle()

if __name__ == '__main__':
    rospy.init_node('localization')
    localization = Localization()
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        localization.Localization()
        rate.sleep()

