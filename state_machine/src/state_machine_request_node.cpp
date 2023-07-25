#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include "state_machine/state_machine_request.h"
#include <unistd.h>
#include <vector>
#include <string.h>
#include <json/json.h>
#include <iostream>  
#include <fstream>  
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

state_machine::state_machine_request msg;
geometry_msgs::PoseStamped pose_stam;
int size;
double Pose_list[2][3];
// std::vector<std::vector<double>> Pose_list;

void readJson()
{
    Json::Reader read;
    Json::Value root;

    std::ifstream in("/home/zsy/demo_ws/src/state_machine/json/request.json",std::ios::binary);
    if(!in.is_open())
    {
        std::cout<<"error opening file\n";
        return;
    }
    if (read.parse(in,root))
    {
        msg.map_control = root["map_control"].asInt();
        msg.navigation_task  = root["navigation_task"].asInt();
        msg.hardware_control = root["hardware_control"].asInt();
        msg.map_id = root["map_id"].asString();
        // size = root["pose"].size();
        // // std::cout<<"poses:"<<size<<std::endl;

        // // 读取目标点数据
        for (int i = 0; i < root["pose"].size(); i++)
        {
            // msg.pose_x[i] = root["pose"][i]["pose_x"].asDouble();
            // msg.pose_y[i] = root["pose"][i]["pose_y"].asDouble();
            // msg.pose_w[i] = root["pose"][i]["pose_w"].asDouble();
            // Pose_list[i][0]= root["pose"][i]["pose_x"].asDouble();
            // Pose_list[i][1]= root["pose"][i]["pose_y"].asDouble();
            // Pose_list[i][2]= root["pose"][i]["pose_w"].asDouble();

        }
    }    
    else
        std::cout<<"parse error \n"<<std::endl;

    in.close();
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"state_machine_request_node");
    ros::Publisher machine_request;
    ros::NodeHandle nh;
    machine_request = nh.advertise<state_machine::state_machine_request>("state_machine/state_machine_request",1);
    ros::Rate loopRate(1);

    while(ros::ok())
    {
        readJson();
        msg.pose_x={-3.52,2.24,-3.34,-3.56,3.06,0.952};
        msg.pose_y={4.27,3.81,1.12,-2.62,-2.34,0.583};
        msg.pose_w={-0.00143,-0.00143,0.00247,0.00247,-0.00143,0.00247};

        machine_request.publish(msg);

        ROS_INFO("state_machine_request Info");

        loopRate.sleep();
    }
    return 0;
}


