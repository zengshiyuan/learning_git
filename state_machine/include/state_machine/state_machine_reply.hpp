#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include "state_machine/state_machine_reply.h"
#include <unistd.h>
#include <vector>

#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

//操作.yaml文件
#include <ros/package.h> 
#include <yaml-cpp/yaml.h>


class state_machine_reply
{
private:
    ros::NodeHandle nh;
    ros::Subscriber map_control_sub,cancle_pub_,navgition_task_sub,goal_pose_pub,hd_pub;
    ros::Publisher machine_reply;
public:
    state_machine_reply(ros::NodeHandle &nh);
    void mapCb(const state_machine::state_machine_reply::ConstPtr &msg);
    void navCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
};



