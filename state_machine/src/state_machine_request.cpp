#include "state_machine/state_machine_request.hpp"

#define NUM 1024
geometry_msgs::PoseStamped goal_tmp;
state_machine::state_machine_reply reply;
// const state_machine::state_machine_request::ConstPtr request_msg;
geometry_msgs::Twist vel_msg;
// geometry_msgs::PoseArray pose_list[];

state_machine_request::state_machine_request(){}

//杀子进程
void state_machine_request::pid()
{
    std::cout<<"pid = "<<getpid()<<std::endl;
    getchar();// linux经常使用， 让程序暂停下来


}

void state_machine_request::operator_node(std::string cmd)
{
    char line[NUM];
	FILE* fp;
	//系统调用
	const char* sysCommand = cmd.data();
	if ((fp = popen(sysCommand, "r")) == NULL)
	{
		std::cout << "错误" << std::endl;
		return;
	}
	//输出
	while (fgets(line, sizeof(line) - 1, fp) != NULL)
	{
		std::cout << line;
	}
	pclose(fp);

}

void state_machine_request::init(ros::NodeHandle &nh)
{
    request_sub = nh.subscribe<state_machine::state_machine_request>("state_machine/state_machine_request",1,&state_machine_request::RequestCallback,this);
    reply_pub = nh.advertise<state_machine::state_machine_reply>("state_machine/state_machine_reply",1);

    navgition_task_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    cancle_pub_ =  nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    point_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_maker",10);
}

int state_machine_request::DoShutdown(int sig)
{
	ROS_INFO("shutting down!");
	ros::shutdown(); 
    exit(sig);
}

//检查地图ID
void state_machine_request::Checkmap(std::string map_id)
{
    // std::string dir_package, dir_package_file;
    // dir_package = ros::package::getPath("");
    // dir_package_file = dir_package + "config/map_id.yaml";
    // YAML::Node config = YAML::LoadFile(dir_package_file);
    // if(NULL)
    // {
    //     reply.map_control = 1;
    //     reply_pub.publish(reply);
    // }
    
}

//切换地图
void state_machine_request::Switchmap(std::string map_id)
{
    const char *map_ID=map_id.data();
    char *path1 = "rosrun map_server map_server";
    strcat(path1,map_ID);
    char *path2 = ".yaml";
    strcat(path1,path2);
    system(path1);
    // system("rosrun map_server map_server "+ map_ID +".yaml");

}

//开始扫描建图
void state_machine_request::Scanmap_begin()
{
    //启动激光雷达建图文件
    system("roslaunch mbot_navigation gmapping_demo.launch");
    // system("bash ${HOME}/kai_ws/src/mobile_robot/call_launch/tool_sh/ekill.sh 2loc_sensor_fusion.launch");
    // system("bash /home/robot/kai_ws/src/mobile_robot/call_launch/tool_sh/call_mapping.sh");
    // system("roslaunch navigation mapping.launch");
    
    
}
//结束扫描建图
void state_machine_request::Scanmap_end()
{
    system("rosrun map_server map_saver -f /home/zsy/demo_ws/change");
    // char *path1,*path2;
    // std::string map;
    // map = request_msg->map_id;
    // const char *map_id = map.data();
    // path1 = "'rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/kai_ws/'" ;
    // std::strcat(path1,map_id);
    // path2 = "' -pbstream_filename=${HOME}/kai_ws/new_map.pbstream -resolution=0.05'";
    // std::strcat(path1,path2);
    // system(path1);

    // system("rosservice call /finish_trajectory 0");
    // system("rosservice call /write_state '{filename: '${HOME}/kai_ws/new_map.pbstream'}'");
    // system("'rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/kai_ws/' + map_id + ' -pbstream_filename=${HOME}/kai_ws/new_map.pbstream -resolution=0.05'");
    // system("bash ${HOME}/kai_ws/src/mobile_robot/call_launch/tool_sh/ekill.sh 1map_sensor_fusion.launch");

}

//接收目标点
void state_machine_request::receive_pose(const state_machine::state_machine_request::ConstPtr request_msg)
{
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    bool once = true;
    //目标点数量是否超出
    // if (NULL)
    // {
    //     std::cout<<"目标点数量已超出，请重新设置目标点！！！"<<std::endl;
    //     reply.navigation_task = 0x81;
    // }
    // ROS_INFO("pose publish");
    
    //可视化目标点
    for (int i =0; i < 6; ++i) {
        if (once) {
            marker.action = visualization_msgs::Marker::DELETEALL;
            once = false;
        } else {
            marker.action = visualization_msgs::Marker::ADD;
        }
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1;
        marker.color.r = 255;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.pose.position.x = request_msg->pose_x[i];
        marker.pose.position.y = request_msg->pose_y[i];
        marker.pose.position.z = 1;

        marker.pose.orientation.w = 1.0;//文字的方向
        marker_array.markers.push_back(marker);
    }
    point_pub.publish(marker_array);
    // ROS_INFO("Goal publish");
    reply_pub.publish(reply);
}

//矩形坐标
void state_machine_request::square_pose(const state_machine::state_machine_request::ConstPtr request_msg)
{
    // //目标点数量是否超出
    // if (NULL)
    // {
    //     std::cout<<"目标点数量已超出，请重新设置目标点！！！"<<std::endl;
    //     reply.navigation_task = 0x81;
    // }
    // else if (NULL)
    // {
    //     std::cout<<"目标点少于四个，请重新设置四个目标点！！！"<<std::endl;
    // }
    // else
    // {
    //     for (size_t i = 0; i < 4; i++)
    //     {
    //         goal_tmp.pose.position = request_msg->poses[i].position;
    //         goal_tmp.pose.orientation = request_msg->poses[i].orientation; 
    //         navgition_task_pub.publish(goal_tmp);
    //     }
    //     ROS_INFO("Goal publish");
        
    // }
    // reply_pub.publish(reply);

}

//开始导航
void state_machine_request::begin_navigation(const state_machine::state_machine_request::ConstPtr &request_msg)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

    // signal(SIGINT, state_machine_request::DoShutdown());
    ROS_INFO("move_base_square.cpp start...");

    //Publisher to manually control the robot (e.g. to stop it, queue_size=5)
    //发布手动控制机器人停止
    // cmdVelPub = node.advertise<geometry_msgs::Twist>(topic, 5);

    ROS_INFO("Waiting for move_base action server...");
    //Wait 60 seconds for the action server to become available
    if(!ac.waitForServer(ros::Duration(60)))
    {
        ROS_INFO("Can't connected to move base server");
        return ;
    }
    ROS_INFO("Connected to move base server");
    ROS_INFO("Starting navigation test");

    //Initialize a counter to track waypoints
    int count = 0;
    //Cycle through the four waypoints
    while( (count < request_msg->pose_x.size()) && (ros::ok()) )
    {   
        //导航任务正在执行中，0x02
        reply.navigation_task =0x02;
        //Intialize the waypoint goal
        move_base_msgs::MoveBaseGoal goal;

        //Use the map frame to define goal poses
        goal.target_pose.header.frame_id = "map";

        //Set the time stamp to "now"
        goal.target_pose.header.stamp = ros::Time::now();

        //Set the goal pose to the i-th waypoint
        // goal.target_pose.pose = pose_list.pose;
        goal.target_pose.pose.position.x = request_msg->pose_x[count];
        goal.target_pose.pose.position.y = request_msg->pose_y[count];
        goal.target_pose.pose.orientation.w = request_msg->pose_w[count];

        //Start the robot moving toward the goal
        //Send the goal pose to the MoveBaseAction server
        ac.sendGoal(goal);

        //Allow 1 minute to get there
        bool finished_within_time = ac.waitForResult(ros::Duration(60));

        //If we dont get there in time, abort the goal
        if(!finished_within_time)
        {
            ac.cancelGoal();
            ROS_INFO("Timed out achieving goal");
        }
        else
        {
            //We made it!
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal succeeded!");
            // reply.navigation_task = 0x04;
        }
        else
        {
            ROS_INFO("The base failed for some reason");
        }
        }
        count += 1;
    }
    //判断全部路径点是否都导航成功
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        reply.navigation_task = 0x04;
    }
    ROS_INFO("move_base_square.cpp end...");
    reply_pub.publish(reply);

}

//暂停导航
void state_machine_request::pulse_navigation(geometry_msgs::Twist msg)
{
    msg.linear.x=0;
    msg.linear.y=0;
    msg.linear.z=0;
    msg.angular.x=0;
    msg.angular.y=0;
    msg.angular.z=0;
    //将电机停止运动
    vel_pub.publish(msg);
    reply.navigation_task = 0x03;
    reply_pub.publish(reply);

}

//停止导航    stop navigation
void state_machine_request::stop_navigation()
{
    ROS_INFO("stop navigation!!!");

    actionlib_msgs::GoalID first_goal;
    cancle_pub_.publish(first_goal);
}

//请求回调函数
void state_machine_request::RequestCallback(const state_machine::state_machine_request::ConstPtr &msg)
{
    switch (msg->map_control)
    {
    case 0:     //待机状态  IDLE
        // ros::spin();
        break;
    case 1:     //check map     检测地图
        {
            state_machine_request::Checkmap(msg->map_id);
            break;
        }
    case 2:     //switch map    切换地图
        {
            state_machine_request::Switchmap(msg->map_id);
            break;
        }
        break;
    case 3:     //scan map begin 
        {
            //启动建图文件、启动机器人node
            state_machine_request::Scanmap_begin();
        }
        break;
    case 4:     //scan map over
        {
            //建图结束，保存地图
            state_machine_request::Scanmap_end();
        }
        break;   
    default:
        break;
    }
    switch (msg->navigation_task)
    {
    case 0:     //待机状态  IDLE
        // ros::spin();
        break;
    case 1:     //传输路径点
        {
            // ROS_INFO("路径点:");

            state_machine_request::receive_pose(msg);
            break;
        }
    case 2:     //传输区域矩形坐标
        {
            state_machine_request::square_pose(msg);
        break;
        }
    case 3:     //begin nav
        state_machine_request::begin_navigation(msg);

        break;
    case 4:     //pluse nav
        state_machine_request::pulse_navigation(vel_msg);
        break;
    case 5:     //stop nav
        state_machine_request::stop_navigation();
        // ROS_INFO("Goal publish");

        break;
    
    default:
        break;
    }
    
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"state_machine_requset");
    ros::NodeHandle nh;

    state_machine_request request; 
    while (ros::ok())
    {
        request.init(nh);
        ROS_INFO("requset");
        ros::spin();    
    }
    
}
