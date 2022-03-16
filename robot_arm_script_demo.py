#! /usr/bin/env python
# -*- coding:= UTF-8 -*-

import sys
import copy
import rospy
import time
import thread
import threading
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point
from tf import transformations
from math import pi
from rospy.msg import deserialize_messages
from std_msgs.msg import String
from std_msgs.msg import Float64
from moveit_commander.conversions import pose_to_list
from gazebo_ros_link_attacher.srv import Attach,AttachRequest,AttachResponse
from opencv_object_tracking.msg import position_publish 

global rivet_state_pub
global attach_srv
global detach_srv
obejct_data_update_flag = 0
obejct_yolo_data_update_flag = 0
desk_metal_high = 0.856818
desk_screw_high = 0.894967

screw_pos = [[1.14,-0.927,0.813],[1.103,-0.927,0.813],[1.068,-0.927,0.813],[1.14,-0.976,0.813],[1.103,-0.976,0.813],[1.068,-0.976,0.813]]
metal_pos = [[1.123,-1.0,0.807],[1.475,-1.46,0.815],[1.275,-1.5,0.815],[1.375,-1.55,0.815],[1.475,-1.6,0.815]]
screw_model_name = ["screw1","screw2","screw3","screw4","screw5","screw6"]
screw_model_link = "screw_body_base_link"
screw_middle_model_name = "screw_middle"
screw_middle_model_link = "metal_middle_base_link"
metal_model_name = ["metal","metal_1","metal_2","metal_3","metal_4"]
metal_model_link = "metal_body_link"
metal_middle_model_name = "metal_middle"
metal_middle_model_link = "metal_middle_base_link"

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_control', anonymous=True)
    global rivet_state_pub
    global attach_srv
    global detach_srv
    rivet_state_pub = rospy.Publisher('/scara_operation/rivet/joint1_position_controller/command', Float64, queue_size=10)
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    detach_srv.wait_for_service()

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name_arm_bgrip = "arm_bgrip"
    group_name_arm_sgrip = "arm_sgrip"
    group_big_grip_name = "big_grip"
    group_small_grip_name = "small_grip"
    move_group_arm_bgrip = moveit_commander.MoveGroupCommander(group_name_arm_bgrip)
    move_group_arm_sgrip = moveit_commander.MoveGroupCommander(group_name_arm_sgrip)
    move_group_big_grip = moveit_commander.MoveGroupCommander(group_big_grip_name)
    move_group_small_grip = moveit_commander.MoveGroupCommander(group_small_grip_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    planning_frame = move_group_arm_bgrip.get_planning_frame()
    print ("============ Planning frame: %s" % planning_frame)
    eef_link = move_group_arm_bgrip.get_end_effector_link()
    print ("============ End effector link: %s" % eef_link)
    group_names = robot.get_group_names()
    print ("============ Available Planning Groups:", robot.get_group_names())
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group_arm_bgrip = move_group_arm_bgrip
    self.move_group_arm_sgrip = move_group_arm_sgrip
    self.move_group_big_grip = move_group_big_grip
    self.move_group_small_grip = move_group_small_grip
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def Big_Grip_State_Setting(self,grip_joint1,grip_joint2):
    move_group = self.move_group_big_grip
    joint_goal = move_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = grip_joint1
    joint_goal[1] = grip_joint2
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def Small_Grip_State_Setting(self,grip_joint1,grip_joint2):
    move_group = self.move_group_small_grip
    joint_goal = move_group.get_current_joint_values()
    print("************************************************")
    print(joint_goal)
    joint_goal[1] = grip_joint1
    joint_goal[2] = grip_joint2
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def Scara_Move_J(self,joint1,joint2,joint3,joint4):
    move_group = self.move_group_arm_bgrip
    joint_goal = move_group.get_current_joint_values()
    joint_list = [joint1,joint2,joint3,joint4]
    count = 0
    for i in joint_list:
      if(i != -100):
        joint_goal[count] = i
      count += 1

    move_group.go(joint_goal, wait=True)
    move_group.stop()
    
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def Scara_Move_P(self,pos_x,pos_y,pos_z,orien_x,orien_y,orien_z,orien_w,grip):
    group_index = 0
    if grip == "big_grip":
      move_group = self.move_group_arm_bgrip
      group_index = 1
    elif grip == "small_grip":
      move_group = self.move_group_arm_sgrip
      group_index = 2
    else:
      print("error grip setting")
      return 0
    print("%f,%f,%f,%f"%(orien_w,orien_x,orien_y,orien_z))
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = orien_w
    pose_goal.orientation.x = orien_x
    pose_goal.orientation.y = orien_y
    pose_goal.orientation.z = orien_z
    pose_goal.position.x = pos_x
    pose_goal.position.y = pos_y
    pose_goal.position.z = pos_z
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = move_group.get_current_pose().pose
    if(group_index == 1):
      for i in range(5):
        if(abs(current_pose.position.x-metal_pos[i][0]) < 0.03):
          if(abs(current_pose.position.y-metal_pos[i][1]) < 0.03):
            self.Detaching_virtual_link(metal_model_name[i],metal_model_link,metal_middle_model_name,metal_middle_model_link)
    elif(group_index == 2):
      print("wkr")
      if(abs(current_pose.position.y - screw_pos[0][1])<0.015):
        for i in range(3):
          if(abs(current_pose.position.x - screw_pos[i][0])<0.02):
            self.Detaching_virtual_link(screw_model_name[i],screw_model_link,screw_middle_model_name,screw_middle_model_link)
      elif (abs(current_pose.position.y - screw_pos[3][1]) < 0.015):
        for i in range(3,6):
          if(abs(current_pose.position.x - screw_pos[i][0])<0.02):
            self.Detaching_virtual_link(screw_model_name[i],screw_model_link,screw_middle_model_name,screw_middle_model_link)
    return all_close(pose_goal, current_pose, 0.01)

  # def scara_move_pos_by_camera(self,object,high,grip,cam):
  # big high : 0 - 0.125
  # small high: 0 - 0.122
  # def scara_move_pos_by_camera(self,pos_x,pos_y,pos_z,orien_w,orien_x,orien_y,orien_z,grip):
  def Scara_Move_Cam(self,high,grip,cam):
    global target_x
    global target_y
    global target_yolo_x
    global target_yolo_y
    global desk_metal_high
    global desk_screw_high
    global obejct_data_update_flag
    global obejct_yolo_data_update_flag
    obejct_data_update_flag = 1
    time.sleep(1)
    group_index = 0
    
    if grip == "big_grip":
      move_group = self.move_group_arm_bgrip
      current_pose = move_group.get_current_pose()
      group_index = 1
    elif grip == "small_grip":
      move_group = self.move_group_arm_sgrip
      current_pose = move_group.get_current_pose()
      group_index = 2
    else:
      print("error grip setting")
      return 0
    if cam == "hsv":
      x = target_x
      y = target_y
      z = desk_metal_high + high
      print("****"*12,"HSV")
    elif cam == "yolo":
      obejct_yolo_data_update_flag = 1
      time.sleep(2)
      x = target_yolo_x
      y = target_yolo_y
      z = desk_screw_high + high
      print("****"*12,"YOLO")
    else:
      print("error grip setting")
      return 0
    
    pose_goal = geometry_msgs.msg.Pose()
    orien = transformations.quaternion_from_euler(0,0,object_angle*3.14/180)
    pose_goal.orientation.w = orien[3]
    pose_goal.orientation.x = orien[0]
    pose_goal.orientation.y = orien[1]
    pose_goal.orientation.z = orien[2]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = move_group.get_current_pose().pose
    if(group_index == 1):
      for i in range(5):
        if(abs(current_pose.position.x-metal_pos[i][0]) < 0.03):
          if(abs(current_pose.position.y-metal_pos[i][1]) < 0.03):
            self.Detaching_virtual_link(metal_model_name[i],metal_model_link,metal_middle_model_name,metal_middle_model_link)
    elif(group_index == 2):
      if(abs(current_pose.position.y - screw_pos[0][1])<0.015):
        for i in range(3):
          if(abs(current_pose.position.x - screw_pos[i][0])<0.02):
            self.Detaching_virtual_link(screw_model_name[i],screw_model_link,screw_middle_model_name,screw_middle_model_link)
      elif (abs(current_pose.position.y - screw_pos[3][1]) < 0.015):
        for i in range(3,6):
          if(abs(current_pose.position.x - screw_pos[i][0])<0.02):
            self.Detaching_virtual_link(screw_model_name[i],screw_model_link,screw_middle_model_name,screw_middle_model_link)
    return all_close(pose_goal, current_pose, 0.01)


  def Rivet_State_Setting(self,distance):
    global rivet_state_pub
    rivet_state_msg = Float64()
    rivet_state_msg.data = distance
    rivet_state_pub.publish(rivet_state_msg)

  def Detaching_virtual_link(slef,model_name_1,link_name_1,model_name_2,link_name_2):
    global detach_srv
    rospy.loginfo("Detaching %s and %s",model_name_1,model_name_2)
    req = AttachRequest()
    req.model_name_1 = model_name_1
    req.link_name_1 = link_name_1
    req.model_name_2 = model_name_2
    req.link_name_2 = link_name_2
    detach_srv.call(req)

  def Attaching_vitrual_link(slef,model_name_1,link_name_1,model_name_2,link_name_2):
    global attach_srv
    rospy.loginfo("Attaching %s and %s",model_name_1,model_name_2)
    req = AttachRequest()
    req.model_name_1 = model_name_1
    req.link_name_1 = link_name_1
    req.model_name_2 = model_name_2
    req.link_name_2 = link_name_2
    attach_srv.call(req)

def delimage_metal_Callback(msg):
  global obejct_data_update_flag
  global target_x
  global target_y
  global object_angle
  if obejct_data_update_flag: 
    target_x = msg.Position_XYZ[0].x
    target_y = msg.Position_XYZ[0].y
    object_angle = msg.angle
    print("---"*10)
    print("target_x %.5f,target_y %.5f, angle %.5f"%(target_x,target_y,object_angle))
    obejct_data_update_flag = 0

def delimage_yolo_Callback(msg):
  global obejct_yolo_data_update_flag
  global target_yolo_x
  global target_yolo_y
  if obejct_yolo_data_update_flag: 
    target_yolo_x = msg.x
    target_yolo_y = msg.y
    print("---"*10)
    print("target_yolo_x %.5f,target_yolo_y %.5f"%(target_yolo_x,target_yolo_y))
    obejct_yolo_data_update_flag = 0

def DelImage_HSV_Metal(a,b):
  rospy.Subscriber("/position_object_two",position_publish,delimage_metal_Callback,queue_size=1)
  rospy.spin()

def DelImage_Yolo_Screw(a,b):
  rospy.Subscriber("/detected_objects_pose",Point,delimage_yolo_Callback,queue_size=1)
  print("This is the yolo node")
  rospy.spin()

def main():
  try:
    print ("")
    print ("----------------------------------------------------------")
    print ("Welcome to the Artificial intelligence virtual simulation platform")
    print ("----------------------------------------------------------")
    print ("Press Ctrl-D to exit at any time")
    print ("")

    global rivet_state_pub
    global attach_srv
    global detach_srv
    global screw_pos
    global metal_pos

    thread.start_new_thread(DelImage_HSV_Metal,(1,2))

    thread.start_new_thread(DelImage_Yolo_Screw,(3,4))

    tutorial = MoveGroupPythonIntefaceTutorial()

    rivet_state_msg = Float64()
    rivet_state_msg.data = 0.0
    rivet_state_pub.publish(rivet_state_msg)


    

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
