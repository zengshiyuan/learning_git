#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include "state_machine/state_machine_request.h"
#include "state_machine/state_machine_reply.h"

#include <unistd.h>
#include <vector>

#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//操作.yaml文件
#include <ros/package.h> 
#include <yaml-cpp/yaml.h>


class state_machine_request
{
private:
    ros::NodeHandle nh;
    ros::Subscriber request_sub;
    ros::Publisher map_control_pub,cancle_pub_,navgition_task_pub;
    ros::Publisher goal_pose_pub,vel_pub,reply_pub;
    ros::Publisher point_pub;

public:
    state_machine_request();
    void init(ros::NodeHandle &nh);
    void RequestCallback(const state_machine::state_machine_request::ConstPtr &msg);
    void Checkmap(std::string map_id);
    void Switchmap(std::string map_id);
    void Scanmap_begin();
    void Scanmap_end();

    void begin_navigation(const state_machine::state_machine_request::ConstPtr &request_msg);
    void receive_pose(const state_machine::state_machine_request::ConstPtr request_msg);
    void square_pose(const state_machine::state_machine_request::ConstPtr request_msg);
    void stop_navigation();
    void pulse_navigation(geometry_msgs::Twist msg);

    void operator_node(std::string cmd);
    void pid();
    int DoShutdown(int sig);
};

