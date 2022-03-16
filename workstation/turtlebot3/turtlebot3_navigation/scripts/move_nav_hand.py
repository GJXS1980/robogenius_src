#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import yaml
import os
import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  


#fileNamePath = os.path.split(os.path.realpath('/root/hg_ws/src/workstation/turtlebot3/turtlebot3_navigation/config/goal.yaml'))[0]
fileNamePath = os.path.split(os.path.realpath('/home/hgsim/hg_ws/src/workstation/turtlebot3/turtlebot3_navigation/config/goal.yaml'))[0]
print(fileNamePath)
#/home/pickle/hpro_ws/src/workstation/turtlebot3/turtlebot3_navigation/config
yamlPath = os.path.join(fileNamePath,'goal_nav.yaml')
print(yamlPath)
#/home/pickle/hpro_ws/src/workstation/turtlebot3/turtlebot3_navigation/config/goal.yaml

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


# 节点初始化 
rospy.init_node('move_test', anonymous=True)  
  
# 订阅move_base服务器的消息  
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction) 
# wkr add
nav_goal_pub = rospy.Publisher('/nav_goal', Twist, queue_size=10) 

rospy.loginfo("Waiting for move_base action server...")  

# 等待连接服务器，5s等待时间限制 
while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
	rospy.loginfo("Connected to move base server") 


while(1):
	key = int(input("Please input the key and choose target: "))
	print(key)
	if key == 1 :
		#target = Pose(Point(-0.60, -0.425, 0.000), Quaternion(0.000, 0.000, 0.000, 1))
		target = Pose(Point(goal1_x, goal1_y, goal1_z), Quaternion(goal_orientation_x, goal_orientation_y, goal_orientation_z, goal_orientation_w))
		goal = MoveBaseGoal()  
		goal.target_pose.pose = target  
		goal.target_pose.header.frame_id = 'map'  
		goal.target_pose.header.stamp = rospy.Time.now()  

		rospy.loginfo("Going to: " + str(target))  
		# 向目标进发  
		move_base.send_goal(goal)  

		# 五分钟时间限制  
		finished_within_time = move_base.wait_for_result(rospy.Duration(300))   

		# 查看是否成功到达  
		if not finished_within_time:  
			move_base.cancel_goal()  
			rospy.loginfo("Timed out achieving goal")  
		else:  
			state = move_base.get_state()  
			if state == GoalStatus.SUCCEEDED:  
				rospy.loginfo("Goal succeeded!")
				nav_msg = Twist()
				nav_msg.linear.x = 1
				nav_goal_pub.publish(nav_msg)
			else:  
			  rospy.loginfo("Goal failed！ ") 
	elif key == 2 :
		#target = Pose(Point(-1, -0.385, 0.000), Quaternion(0.000,0.000,0.000,1))
		target = Pose(Point(goal2_x, goal2_y, goal2_z), Quaternion(goal_orientation_x, goal_orientation_y, goal_orientation_z, goal_orientation_w))
		goal = MoveBaseGoal()  
		goal.target_pose.pose = target  
		goal.target_pose.header.frame_id = 'map'  
		goal.target_pose.header.stamp = rospy.Time.now()  

		rospy.loginfo("Going to: " + str(target))  
		# 向目标进发  
		move_base.send_goal(goal)  

		# 五分钟时间限制  
		finished_within_time = move_base.wait_for_result(rospy.Duration(300))   

		# 查看是否成功到达  
		if not finished_within_time:  
			move_base.cancel_goal()  
			rospy.loginfo("Timed out achieving goal")  
		else:  
			state = move_base.get_state()  
			if state == GoalStatus.SUCCEEDED:  
				rospy.loginfo("Goal succeeded!")
				nav_msg = Twist()
				nav_msg.linear.x = 2
				nav_goal_pub.publish(nav_msg)
			else:  
			  rospy.loginfo("Goal failed！ ")
	elif key == 3 :
		#target = Pose(Point(0.000, 0.000, 0.000), Quaternion(0.000,0.000,0.000,1))
		target = Pose(Point(goal3_x, goal3_y, goal3_z), Quaternion(goal_orientation_x, goal_orientation_y, goal_orientation_z, goal_orientation_w))
		goal = MoveBaseGoal()  
		goal.target_pose.pose = target  
		goal.target_pose.header.frame_id = 'map'  
		goal.target_pose.header.stamp = rospy.Time.now()  

		rospy.loginfo("Going to: " + str(target))  
		# 向目标进发  
		move_base.send_goal(goal)  

		# 五分钟时间限制  
		finished_within_time = move_base.wait_for_result(rospy.Duration(300))   

		# 查看是否成功到达  
		if not finished_within_time:  
			move_base.cancel_goal()  
			rospy.loginfo("Timed out achieving goal")  
		else:  
			state = move_base.get_state()  
			if state == GoalStatus.SUCCEEDED:  
				rospy.loginfo("Goal succeeded!")
			else:  
			  rospy.loginfo("Goal failed！ ")
	else:
		exit() 


# 设定目标点 

#target = Pose(Point(0.000, 0.000, 0.000), Quaternion(-0.0001518, 0.000200304, -0.0129676, 0.997))  
#target = Pose(Point(-0.60, -0.47, 0.000), Quaternion(-0.0001518, 0.000200304, -0.0129676, 0.997))  

 

#raw_input()
# 设定目标点  
#target = Pose(Point(-0.75, -0.47, 0.000), Quaternion(-0.0001518, 0.000200304, -0.0129676, 0.997))  #goal = MoveBaseGoal()  
#goal.target_pose.pose = target  
#goal.target_pose.header.frame_id = 'map'  
#goal.target_pose.header.stamp = rospy.Time.now()  

#rospy.loginfo("Going to: " + str(target))  

# 向目标进发  
#move_base.send_goal(goal)  

# 五分钟时间限制  
#finished_within_time = move_base.wait_for_result(rospy.Duration(300))   

# 查看是否成功到达  
#if not finished_within_time:  
#    move_base.cancel_goal()  
#    rospy.loginfo("Timed out achieving goal")  
#else:  
#    state = move_base.get_state()  
#    if state == GoalStatus.SUCCEEDED:  
#        rospy.loginfo("Goal succeeded!")
#    else:  
#      rospy.loginfo("Goal failed！ ")  


