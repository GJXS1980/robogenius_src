相对于 world 

```
xyz = "1.5835 -1.19925 0.619"
rpy = "0 0 3.14"
```



- scara_base.STL
- scara_1_link.STL
  - prismatic
  - effector --- 500
- scara_2_link.STL
  - revolute
  - effector --- 100
- scara_3_link.STL
  - revolute
  - effector --- 100
- scara_4_link.STL
  - revolute
  - effector --- 100
- scara_dualgrip_base.STL
  - ~~revolute --- setting error~~
  - fixed --- rebuild for right
- scara_dualgrip_revolute_link.STL
  - revolute
  - effector --- 100
- scara_dualgrip_small_leftgrip.STL
  - prismatic
  - effector --- 100
  - range -0.1 - 0.007
- scara_dualgrip_small_rightgrip.STL
  - prismatic
  - effector --- 100
  - range -0.1 - 0.007
- scara_dualgrip_big_leftgrip.STL
  - prismatic
  - effector --- 100
  - range -0.1 - 0.007
- scara_dualgrip_big_rightgrip.STL
  - prismatic
  - effector --- 100
  - range -0.1 - 0.007



夹爪配置

```
 <gazebo>
   <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
        <arm>
           <arm_name>scara</arm_name>
           <palm_link> scara_dualgrip_base  </palm_link>
           <gripper_link> scara_dualgrip_big_leftgrip </gripper_link>
           <gripper_link> scara_dualgrip_big_rightgrip </gripper_link>
        </arm>
        <arm>
           <arm_name>scara_small</arm_name>
           <palm_link> scara_dualgrip_revolute_link  </palm_link>
           <gripper_link> scara_dualgrip_small_leftgrip </gripper_link>
           <gripper_link> scara_dualgrip_small_rightgrip </gripper_link>
        </arm>
       <forces_angle_tolerance>100</forces_angle_tolerance>
       <update_rate>4</update_rate>11
       <grip_count_threshold>1</grip_count_threshold>
       <max_grip_count>8</max_grip_count>
       <release_tolerance>0.005</release_tolerance>
       <disable_collisions_on_attach>true</disable_collisions_on_attach>
       <contact_topic>__default_topic__</contact_topic>
    </plugin>
</gazebo> 
<gazebo>
  <gripper name="grasping">
    <grasp_check>
     <attach_steps>10</attach_steps>
     <detach_steps>10</detach_steps>
     <min_contact_count>1</min_contact_count>
    </grasp_check>
      <gripper_link>scara_dualgrip_big_leftgrip</gripper_link>
      <gripper_link>scara_dualgrip_big_rightgrip</gripper_link>
      <palm_link>scara_dualgrip_base</palm_link>
  </gripper>
</gazebo>
```



距离侧方钣金高度范围

- 0.856818 - 0.981241
