#!/usr/bin/env python
import rospy
import time
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from link_controller.msg import link_ctrl


'''
scara_operation    scara_workspace_base
screw_base         metal_scara_base_link
screw_middle       metal_middle_base_link

screw1-6           screw_body_base_link

'''


if __name__ == '__main__':
    rospy.init_node('link_controller_one')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching metal and metal_middle")
    req = AttachRequest()
    req.model_name_1 = "metal"
    req.link_name_1 = "metal_body_link"
    req.model_name_2 = "metal_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching metal_1 and metal_middle")
    req = AttachRequest()
    req.model_name_1 = "metal_1"
    req.link_name_1 = "metal_body_link"
    req.model_name_2 = "metal_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching metal2 and metal_middle")
    req = AttachRequest()
    req.model_name_1 = "metal_2"
    req.link_name_1 = "metal_body_link"
    req.model_name_2 = "metal_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching metal3 and metal_middle")
    req = AttachRequest()
    req.model_name_1 = "metal_3"
    req.link_name_1 = "metal_body_link"
    req.model_name_2 = "metal_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching metal4 and metal_middle")
    req = AttachRequest()
    req.model_name_1 = "metal_4"
    req.link_name_1 = "metal_body_link"
    req.model_name_2 = "metal_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching metal_middle and metal_base")
    req = AttachRequest()
    req.model_name_1 = "metal_middle"
    req.link_name_1 = "metal_middle_base_link"
    req.model_name_2 = "metal_base"
    req.link_name_2 = "metal_scara_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching metal_base and scara_operation")
    req = AttachRequest()
    req.model_name_1 = "metal_base"
    req.link_name_1 = "metal_scara_base_link"
    req.model_name_2 = "scara_operation"
    req.link_name_2 = "scara_workspace_base"

    attach_srv.call(req)

    rospy.loginfo("Attaching screw_base and screw_middle")
    req = AttachRequest()
    req.model_name_1 = "screw_base"
    req.link_name_1 = "metal_scara_base_link"
    req.model_name_2 = "screw_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching screw1 and screw_middle")
    req = AttachRequest()
    req.model_name_1 = "screw1"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching screw2 and screw_middle")
    req = AttachRequest()
    req.model_name_1 = "screw2"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching screw3 and screw_middle")
    req = AttachRequest()
    req.model_name_1 = "screw3"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching screw4 and screw_middle")
    req = AttachRequest()
    req.model_name_1 = "screw4"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching screw5 and screw_middle")
    req = AttachRequest()
    req.model_name_1 = "screw5"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching screw6 and screw_middle")
    req = AttachRequest()
    req.model_name_1 = "screw6"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    # Material desk ------- metal
    rospy.loginfo("Attaching metal_base_material and material_desk")
    req = AttachRequest()
    req.model_name_1 = "metal_base_material"
    req.link_name_1 = "metal_scara_base_link"
    req.model_name_2 = "material_desk"
    req.link_name_2 = "desk_base"

    attach_srv.call(req)

    rospy.loginfo("Attaching metal_middle_material and metal_base_material")
    req = AttachRequest()
    req.model_name_1 = "metal_middle_material"
    req.link_name_1 = "metal_middle_base_link"
    req.model_name_2 = "metal_base_material"
    req.link_name_2 = "metal_scara_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching metal_material and metal_middle_material")
    req = AttachRequest()
    req.model_name_1 = "metal_material"
    req.link_name_1 = "metal_body_link"
    req.model_name_2 = "metal_middle_material"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)


    # Material desk ------- screw
    '''rospy.loginfo("Attaching screw_base_material and material_desk")
    req = AttachRequest()
    req.model_name_1 = "screw_base_material"
    req.link_name_1 = "metal_scara_base_link"
    req.model_name_2 = "material_desk"
    req.link_name_2 = "desk_base"

    attach_srv.call(req)

    rospy.loginfo("Attaching screw_middle_material and screw_base_material")
    req = AttachRequest()
    req.model_name_1 = "screw_middle_material"
    req.link_name_1 = "metal_middle_base_link"
    req.model_name_2 = "screw_base_material"
    req.link_name_2 = "metal_scara_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching material_screw1 and screw_middle_material")
    req = AttachRequest()
    req.model_name_1 = "material_screw1"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle_material"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching material_screw2 and screw_middle_material")
    req = AttachRequest()
    req.model_name_1 = "material_screw2"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle_material"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching material_screw3 and screw_middle_material")
    req = AttachRequest()
    req.model_name_1 = "material_screw3"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle_material"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching material_screw4 and screw_middle_material")
    req = AttachRequest()
    req.model_name_1 = "material_screw4"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle_material"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching material_screw5 and screw_middle_material")
    req = AttachRequest()
    req.model_name_1 = "material_screw5"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle_material"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)

    rospy.loginfo("Attaching material_screw6 and screw_middle_material")
    req = AttachRequest()
    req.model_name_1 = "material_screw6"
    req.link_name_1 = "screw_body_base_link"
    req.model_name_2 = "screw_middle_material"
    req.link_name_2 = "metal_middle_base_link"

    attach_srv.call(req)'''
