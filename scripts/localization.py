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
from numpy.random import randn
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Quaternion,PoseWithCovarianceStamped,PoseArray,Pose
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

PARTICLE_NUM = 100

class Localization:
    def __init__(self):
        #publisher & subscriber
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
        #self.base_sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.BaseCB)
        self.particle_pub = rospy.Publisher('/e_particles',PoseArray,queue_size=1)
        self.marker_pub = rospy.Publisher('/robot_pose',Marker,queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.LaserCB)
        self.robot_pose_sub= rospy.Subscriber('/gazebo/model_states',LaserScan,self.visualizeSimulatedRobotPoseCB)
        #for emergency stop
        self.particles = []
        self.initParticle()

#---------------------------CallBack
    def BaseCB(self,pose):
        self.robot_pose_x = pose.pose.pose.position.x
        self.robot_pose_y = pose.pose.pose.position.y
        if self.emergency_stop_flg == False:
            print 'remember now pose'
            self.before_stop_pose = pose
            
    def LaserCB(self,laser_scan):
        #self.centor_laser_dist = laser_scan.ranges[405]
        #self.laser_update_count += 1#use for detecting emergency stop
        a = 0

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
        print 114514
        self.marker_pub.publish(marker_data)

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
        self.marker_pub.publish(marker_data)
        
    def Localization(self):
        print 111
        mymap = cv2.imread("/home/demulab/map/mymap.pgm",0)
        cv2.imshow("aaa",mymap)
        cv2.waitKey(1)
        self.visualizeParticle()
        self.visualizeLocalizedRobotPose()

if __name__ == '__main__':
    rospy.init_node('localization')
    localization = Localization()
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        localization.Localization()
        rate.sleep()

