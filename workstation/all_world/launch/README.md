## 1 gazebo_model.launch

1. 用于查看单一模型，校验模型专用



## 2 gazebo_nav.launch

1. 加载地图配置文件
2. 加载 demo_gazebo.launch 启动文件
3. 加载 agv_navigation.launch 启动文件
4. 加载 move_nav.launch  启动文件



## 3 gazebo_pick.launch

1. 加载地图配置文件
2. 加载 demo_gazebo.launch 启动文件
3. 加载 agv_navigation.launch 启动文件
4. 加载 move_nav.launch 启动文件
5. 加载 object_detection.launch 启动文件
6. 加载 pick_place.launch 启动文件



## 4 gazebo_spawn.launch

1. 设置钣金件参考变量
2. 设置 Gazebo 仿真环境暂停变量
3. 加载 Gazebo 仿真环境空地图
4. 模型生成节点 spawn_model_all_world_base 生成围栏工作场景
5. 模型生成节点 spawn_model_material_desk 生成物料台
6. 模型生成节点 spawn_model_top_material 生成立体仓库
7. 模型生成节点 spawn_model_coke_can 生成可乐罐
8. 加载 robot_description scara操作台
9. 模型生成节点 spawn_model_metal_base 加载钣金件底座
10. 模型生成节点 spawn_model_metal_middle 加载钣金件夹层
11. 模型生成节点 spawn_model_metal 加载钣金件
12. 模型生成节点 spawn_model_screw_base 加载螺丝底座
13. 模型生成节点 spawn_model_screw_middle 加载螺丝夹层
14. 模型生成节点 spawn_model_screw1 加载螺丝1
15. 模型生成节点 spawn_model_screw2 加载螺丝2
16. 模型生成节点 spawn_model_screw3加载螺丝3
17. 模型生成节点 spawn_model_screw4 加载螺丝4
18. 模型生成节点 spawn_model_screw5 加载螺丝5
19. 模型生成节点 spawn_model_screw6 加载螺丝6
20. 模型生成节点 spawn_model_nut 加载螺母
21. 模型生成节点 spawn_model_metal_1 加载钣金件1
22. 模型生成节点 spawn_model_metal_2 加载钣金件2
23. 模型生成节点 spawn_model_metal_3 加载钣金件3
24. 模型生成节点 spawn_model_metal_4 加载钣金件4



## 5 gazebo_spawn_operation.launch

1. 



## 6 gazebo_workstation.launch

1. 用于查看世界模型，校验仿真场景
2. 加载机器人参数
3. 加载 workstation 世界



## 7 gazebo_world_turtlebot.launch

1. 加载世界模型
2. 加载 turtlebot 机器人
3. 用于 turtlebot 机器人的仿真