#! /usr/bin/env python
# -*- coding:UTF-8 -*-


import rospy
import time
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == "__main__":
    # 建立虚拟关节连接
    # rospy.init_node("link_controller_user_attch")
    # rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    # attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
    # attach_srv.wait_for_service()

    # rospy.loginfo("Attaching model1_name and model2_name")
    # req = AttachRequest()
    # req.model1_name = "model1_name"
    # req.link1_name = "model1_link_name"
    # req.model2_name = "model2_name"
    # req.link2_name = "model2_link_name"

    # attach_srv.call(req)

    # 取消虚拟关节连接
    # rospy.init_node("link_controller_user_detach")
    # rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    # attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
    # attach_srv.wait_for_service()
    # rospy.loginfo("Attaching model1_name and model2_name")
    # req = AttachRequest()
    # req.model1_name = "model1_name"
    # req.link1_name = "model1_link_name"
    # req.model2_name = "model2_name"
    # req.link2_name = "model2_link_name"

    # attach_srv.call(req)

