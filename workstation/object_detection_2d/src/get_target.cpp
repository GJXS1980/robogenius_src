#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>
#include "opencv_object_tracking/position_publish.h"
#include "opencv_object_tracking/agv2robot.h"
#include "opencv_object_tracking/robot2agv.h"
#include "opencv_object_tracking/target.h"
#include <math.h>  
#include <tf/transform_listener.h>
#include <iostream>
using namespace std;
using namespace Eigen;
float X,Y,Z;
int color_receive;
int obj_found= 0;
float object_angle;

void chatterCallback(const opencv_object_tracking::position_publish& msg)
{
	Y=msg.Position_XYZ[0].y;
	X=msg.Position_XYZ[0].x;
	Z=msg.Position_XYZ[0].z;
	cout << X << " " << Y << " " << Z <<endl;
	obj_found= msg.counter;
	color_receive=msg.color_type;
	object_angle=msg.angle;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_target");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/position_object", 10, chatterCallback);
  ros::Publisher target_pub = n.advertise<opencv_object_tracking::target>("/target_xy",10);
  ros::Publisher xy_pub = n.advertise<geometry_msgs::Twist>("/target_xy_data", 10);
  ros::Rate loop_rate(10);
  tf::TransformListener listener;
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "cobot_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  //obj to cam 
  MatrixXd T1(4, 4);
  //cam to base 
  MatrixXd T2(4, 4);
  float x1,y1,z1,qx1,qy1,qz1,qw1; 
  // obj to base
  MatrixXd T(4, 4);

while (ros::ok())
{
  tf::StampedTransform transform;
      try
      {
        listener.lookupTransform("/base_link", "/realsense_camera_link",ros::Time(0), transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

		T1<<1,0,0,X,
			0,1,0,Y,
			0,0,1,Z,
			0,0,0,1;

    x1=transform.getOrigin().x();
    y1=transform.getOrigin().y();
    z1=transform.getOrigin().z();
    qx1=transform.getRotation().getX();
    qy1=transform.getRotation().getY();
    qz1=transform.getRotation().getZ();
    qw1=transform.getRotation().getW();
    T2<<1 - 2*qy1*qy1 - 2*qz1*qz1,	2*qx1*qy1 - 2*qz1*qw1,	2*qx1*qz1 + 2*qy1*qw1,x1,
      2*qx1*qy1 + 2*qz1*qw1,	1 - 2*qx1*qx1 - 2*qz1*qz1,	2*qy1*qz1 - 2*qx1*qw1,y1,
      2*qx1*qz1 - 2*qy1*qw1,	2*qy1*qz1 + 2*qx1*qw1,	1 - 2*qx1*qx1 - 2*qy1*qy1,z1,
      0,0,0,1;
    T=T2*T1;

    //cout<<"qx1="<<qx1<<endl;
    //cout<<"qy1="<<qy1<<endl;
    //cout<<"qz1="<<qz1<<endl;
    //cout<<"qw1="<<qw1<<endl;

    //cout<<"T1="<<T1<<endl;
    //cout<<"T2="<<T2<<endl;
    cout<<"T="<<T<<endl;



if(obj_found==1)
{
  MatrixXd R(3, 3);
  MatrixXd R1(3, 3);
  MatrixXd R2(3, 3);
  // 夹爪朝下
  R1<<0,-1,0,
            -1,0,0,
            0,0,-1;
  //绕X轴旋转逆时针旋转（realsense的X轴朝外）  若改回对末端进行规划，R2应改绕Z轴旋转
   object_angle=(object_angle-90)*3.1415926/180;
   cout<<"angle="<<object_angle*180/3.1415926<<endl;
   R2<< cos(object_angle),-sin(object_angle),0,
              sin(object_angle),cos(object_angle),0,
              0,0,1;
             
    R=R2;
    cout<<"R2="<<R2<<endl;

    //rot to quat
    float  qx2,qy2,qz2,qw2;
    qw2 = 0.5 * sqrt(1 + R(0,0) + R(1,1) + R(2,2));
    qx2=(R(2,1)-R(1,2))/(4*qw2);
    qy2=(R(0,2)-R(2,0))/(4*qw2);
    qz2= (R(1,0)-R(0,1))/(4*qw2);

//z 0.74971  x -0.581568 y -0.000979
  //geometry_msgs::Pose target_pose1;
  //target_pose1.orientation.w =-0.0011;
  //target_pose1.orientation.x =-0.707;
  ////target_pose1.orientation.y = 0.707;
  //target_pose1.orientation.z=0.0011;
  //target_pose1.orientation.w =qw2;
  //target_pose1.orientation.x =qx2;
  //target_pose1.orientation.y = qy2;
  //target_pose1.orientation.z=qz2;
  //target_pose1.position.x = T(0,3);
  ///target_pose1.position.y = T(1,3);
  ////target_pose1.position.z = T(2,3)+0.5;

  opencv_object_tracking::target target_msg;
  target_msg.target_center_x = T(0,3);
  target_msg.target_center_y = T(1,3);
  //target_msg.target_center_z = T(2,3);
  target_pub.publish(target_msg);

  geometry_msgs::Twist data_msg;
  data_msg.linear.x = T(0,3);
  data_msg.linear.y = T(1,3);
  data_msg.linear.z = obj_found;
  xy_pub.publish(data_msg);

  //move_group.setPoseTarget(target_pose1);
  //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //move_group.plan(my_plan);
  //move_group.execute(my_plan);
  //cout<<"done"<<endl;
  //  break;
  }
 cout<<"wait........"<<endl;
 ros::spinOnce();
loop_rate.sleep();
}
 
  return 0;
}
