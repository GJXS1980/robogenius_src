#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy 
import yaml
import os
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

fileNamePath = os.path.split(os.path.realpath('/home/pickle/melodic_package/hg_ws/src/workstation/turtlebot3/turtlebot3_navigation/config/goal.yaml'))[0]
# print(fileNamePath)
#/home/pickle/hpro_ws/src/workstation/turtlebot3/turtlebot3_navigation/config
yamlPath = os.path.join(fileNamePath,'goal_nav.yaml')
# print(yamlPath)
f = open(yamlPath,'r')
cont = f.read()
x = yaml.load(cont)
goal1_x = x['Material_table']['goal_x']
goal1_y = x['Material_table']['goal_y']
goal1_z = x['Material_table']['goal_z']

goal2_x = x['Stereoscopic_warehouse']['goal_x']
goal2_y = x['Stereoscopic_warehouse']['goal_y']
goal2_z = x['Stereoscopic_warehouse']['goal_z']

goal3_x = x['Origin']['goal_x']
goal3_y = x['Origin']['goal_y']
goal3_z = x['Origin']['goal_z']

goal_orientation_x = x['Material_table']['Orientation_x']
goal_orientation_y = x['Material_table']['Orientation_y']
goal_orientation_z = x['Material_table']['Orientation_z']
goal_orientation_w = x['Material_table']['Orientation_w']

nav_goal_points = 0
msg = Pose()

def doMsg(msg):
    global nav_goal_points
    rospy.loginfo("nav_goal_point: %d",msg.linear.x)
    nav_goal_points = msg.linear.x

def case1():
    global msg
    rospy.loginfo("nav_goal is set to 1")
    msg.position.x = goal1_x
    msg.position.y = goal1_y
    msg.position.z = goal1_z
    msg.orientation.x = goal_orientation_x
    msg.orientation.y = goal_orientation_y
    msg.orientation.z = goal_orientation_z
    msg.orientation.w = goal_orientation_w

def case2():
    global msg
    rospy.loginfo("nav_goal is set to 2")
    msg.position.x = goal2_x
    msg.position.y = goal2_y
    msg.position.z = goal2_z
    msg.orientation.x = goal_orientation_x
    msg.orientation.y = goal_orientation_y
    msg.orientation.z = goal_orientation_z
    msg.orientation.w = goal_orientation_w

def case3():
    global msg
    rospy.loginfo("nav_goal is set to 3")
    msg.position.x = goal3_x
    msg.position.y = goal3_y
    msg.position.z = goal3_z
    msg.orientation.x = goal_orientation_x
    msg.orientation.y = goal_orientation_y
    msg.orientation.z = goal_orientation_z
    msg.orientation.w = goal_orientation_w

def default():
    rospy.loginfo("No Way")

if __name__ == "__main__":
    rospy.init_node("nav_goal_info")
    pub = rospy.Publisher("/nav_goal_info",Pose,queue_size=10)
    sub = rospy.Subscriber("/nav_goal",Twist,doMsg,queue_size=10)
    rate = rospy.Rate(10)
    switch = {'case1':case1,'case2':case2,'case3':case3}
    while not rospy.is_shutdown():
        switch.get(nav_goal_points,default)()
        pub.publish(msg)
        nav_goal_points = 0
        # rospy.loginfo("I am still working !")
        rate.sleep()
    # sub = rospy.Subscriber("/gazebo/model_states",ModelStates,doMsg,queue_size=10)
    rospy.spin()