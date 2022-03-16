#!/usr/bin/env python
import rospy
import time
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

metal_ref = [1.646,-0.857,0.812]
screw_model_link = "screw_body_base_link"
metal_model_link = "metal_body_link"
connect_flag = 0
attach_record = []

'''
['ground_plane', 'all_world_base', 'material_screw3', 'material_screw4', 'metal', 'material_screw2', 
'metal_middle', 'top_material', 'metal_base', 'screw_middle', 'metal_material', 'material_screw1', 
'screw1', 'material_screw5', 'nut', 'metal_middle_material', 'screw_base_material', 'metal_base_material', 
'screw5', 'screw_middle_material', 'screw2', 'screw4', 'scara_operation', 'material_desk', 'metal_1', 'screw6', 
'metal_4', 'screw3', 'material_screw6', 'metal_3', 'metal_2', 'screw_base', 'robot']

'''

def Modelstate_Callback(msg):
    global metal_ref
    global screw_model_link
    global metal_model_link
    global attach_srv
    global connect_flag
    global attach_record
    metal_model_list = []
    metal_model_dict={}
    screw_model_list = []
    screw_model_dict={}
    screw_model_temp_dict = {}
    count = 0
    for model_str in msg.name:
        count += 1
        if(model_str.find('metal') != -1):
            if len(model_str) < 8:
                metal_model_list.append(model_str)
                metal_model_dict[model_str] = count-1
        if(model_str.find("screw") != -1):
            if len(model_str) < 7:
                screw_model_list.append(model_str)
                screw_model_temp_dict[model_str] = count-1
    if(len(attach_record) == 0):
        screw_model_dict = screw_model_temp_dict
    else:
        screw_model_dict = screw_model_temp_dict
        for key_record in attach_record:
            del screw_model_dict[key_record]
    for metal_key in metal_model_dict:
        if((abs(msg.pose[metal_model_dict[metal_key]].position.x-metal_ref[0])<0.015) and (abs(msg.pose[metal_model_dict[metal_key]].position.y-metal_ref[1])<0.015)):
            # print("*******"*10)
            for screw_key in screw_model_dict:
                if( (abs(msg.pose[screw_model_dict[screw_key]].position.x-msg.pose[metal_model_dict[metal_key]].position.x)<0.03) and (abs(msg.pose[screw_model_dict[screw_key]].position.y-msg.pose[metal_model_dict[metal_key]].position.y)<0.03)):
                    connect_flag += 1
                    if((connect_flag > 50) and (abs(msg.pose[screw_model_dict[screw_key]].position.z - 0.82)<0.002)):
                        print("-------"*10)
                        rospy.loginfo("Attaching %s and %s",screw_key,metal_key)
                        req = AttachRequest()
                        req.model_name_1 = screw_key
                        req.link_name_1 = screw_model_link
                        req.model_name_2 = metal_key
                        req.link_name_2 = metal_model_link
                        attach_srv.call(req)
                        connect_flag = 0
                        attach_record.append(screw_key)
                        break


if __name__ == "__main__":
    rospy.init_node("assembly_controller")
    rospy.loginfo("This is a controller for listenning and control the object for connect")
    rospy.Subscriber("/gazebo/model_states",ModelStates,Modelstate_Callback,queue_size=10)
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.spin()