#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include "gazebo_controller/yaml-cpp/yaml.h"
using namespace std;

double robot_PO[7] = {0};
double nav_goal[7] = {0};
int nav_goal_points = 0;

void doMsg(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    int modelcount = msg->name.size();
    for(int modelid = 0;modelid<modelcount;modelid++)
    {
        if(msg->name[modelid] == "robot")
        {
            geometry_msgs::Pose pose = msg->pose[modelid];
            robot_PO[0] = pose.position.x;
            robot_PO[1] = pose.position.y;
            robot_PO[2] = pose.position.z;
            robot_PO[3] = pose.orientation.x;
            robot_PO[4] = pose.orientation.y;
            robot_PO[5] = pose.orientation.z;
            robot_PO[6] = pose.orientation.w;
            // ROS_INFO("%d is robot",modelid);
            // geometry_msgs::Pose pose = msg->pose[modelid];
            // ROS_INFO("x=%.2f,y=%.2f,z=%.2f",pose.position.x,pose.position.y,pose.position.z);
            // ROS_INFO("x=%.2f,y=%.2f,z=%.2f,w=%.2f",pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
            // geometry_msgs::Twist twist = msg->twist[modelid];
        }
    }
}

void doInfo(const geometry_msgs::PoseConstPtr &msg)
{
    nav_goal[0] = msg->position.x;
    nav_goal[1] = msg->position.y;
    nav_goal[2] = msg->position.z;
    nav_goal[3] = msg->orientation.x;
    nav_goal[4] = msg->orientation.y;
    nav_goal[5] = msg->orientation.z;
    nav_goal[6] = msg->orientation.w;
    // ROS_INFO("x=%.2f,y=%.2f,z=%.2f",msg->position.x,msg->position.y,msg->position.z);
}

void doNavGoal(const geometry_msgs::TwistConstPtr msg)
{
    nav_goal_points = msg->linear.x;
    ROS_INFO("nav_goal_points %d",nav_goal_points);
}

int main(int argc,char*argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"get_model_states");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",10,doMsg);
    ros::Subscriber sub_model_info = n.subscribe<geometry_msgs::Pose>("/nav_goal_info",10,doInfo); 
    ros::Subscriber sub_nav_state = n.subscribe<geometry_msgs::Twist>("/nav_goal",10,doNavGoal);
    // YAML::Node config = YAML::LoadFile("/home/pickle/melodic_package/hg_ws/src/workstation/turtlebot3/turtlebot3_navigation/config/goal_nav.yaml");
    while(ros::ok())
    {
        if(nav_goal_points != 0)
        {
            int error_flag = 0;
            for(int i=0;i<3;i++)
            {
                // cout << robot_PO[i] << " " <<nav_goal[i] << endl;
                // cout << i << "  " << robot_PO[i]-nav_goal[i] << endl;
                if(abs(robot_PO[i]-nav_goal[i])>0.1)
                {
                    error_flag++;
                    // cout << i <<endl;
                }
            }
            // cout << error_flag <<endl;
            if(error_flag == 0)
            {
                cout << "robot navigation ok! "<< endl;
            }
            // else
            // {
            //     cout << "robot navigation error! "<< endl;
            // }
        }
        nav_goal_points = 0;
        // for(YAML::const_iterator it= config["Material_table"].begin(); it != config["Material_table"].end();++it)
        // {
        //     cout << it->first.as<string>() << ":" << it->second.as<int>() << endl;
        // }
        // cout << config["Material_table"]["goal_x"];
        // cout << "Material_table goal_x: " << config["Material_table"]["goal_x"].as<float>() << endl;
        ros::spinOnce();
    }
    return 0;
}