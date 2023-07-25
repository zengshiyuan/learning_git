#include "ros/ros.h"
#include <iostream>
#include <std_msgs/String.h>
#include "state_machine/state_machine_reply.h"
#include <unistd.h>
#include <vector>
#include <string.h>
#include <json/json.h>
#include <iostream>  
#include <fstream>  

void replysub_CB(const state_machine::state_machine_reply::ConstPtr &msg)
{
    Json::Value root;
    root["map_control"] = Json::Value(msg->map_control);
    root["navigation"] = Json::Value(msg->navigation_task);

    //缩进输出  
	std::cout << "StyledWriter:" << std::endl;
	Json::StyledWriter sw;
	// std::cout << sw.write(root) << std::endl << std::endl;
 
	//输出到文件  
	std::ofstream os;
	os.open("/home/zsy/demo_ws/src/state_machine/json/reply.json", std::ios::out | std::ios::app);
	if (!os.is_open())
		std::cout << "error：can not find or create the file which named \" demo.json\"." << std::endl;
	os << sw.write(root);
	os.close();

}

int main(int argc, char **argv)
{

    ros::init(argc,argv,"state_machine_reply_node");
    ros::NodeHandle nh;

    ros::Subscriber machine_reply;
    machine_reply = nh.subscribe<state_machine::state_machine_reply>("state_machine/state_machine_reply",1,&replysub_CB);

    ros::Rate loopRate(1);
    while(ros::ok())
    {
        
        ROS_INFO("state_machine_reply Info");
        loopRate.sleep();
    }
    return 0;
}


