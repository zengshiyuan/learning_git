#include "apriltag_ros/apriltag_location.hpp"

namespace apriltag_location{
ApriltagLocation::ApriltagLocation()
{}
void ApriltagLocation::init(ros::NodeHandle nh,ros::NodeHandle nh_p)
{
    vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    tag_center_sub_  = nh.subscribe<apriltag_ros::ApriltagCenter>("/apriltag_ros_continuous_node/tag_center",1,&ApriltagLocation::centerCB, this);
    
    // dynamic_reconfigure::Server<apriltag_ros::apriltag_rosConfig> server;
    // dynamic_reconfigure::Server<apriltag_ros::apriltag_rosConfig>::CallbackType f;
    // f = boost::bind(&ApriltagLocation::dynamicCB,_1,_2);
    // server.setCallback(f);
    
    nh_p.param("pid_x_kp",pid_x.Kp,pid_x.Kp);
    nh_p.param("pid_x_ki",pid_x.Ki,pid_x.Ki);
    nh_p.param("pid_x_kd",pid_x.Kd,pid_x.Kd);

    nh_p.param("pid_yaw_kp",pid_yaw.Kp,pid_yaw.Kp);
    nh_p.param("pid_yaw_ki",pid_yaw.Ki,pid_yaw.Ki);
    nh_p.param("pid_yaw_kd",pid_yaw.Kd,pid_yaw.Kd);

    nh_p.param("pid_lin_Kp",pid_lin.Kp,pid_lin.Kp);
    nh_p.param("pid_lin_ki",pid_lin.Ki,pid_lin.Ki);
    nh_p.param("pid_lin_kd",pid_lin.Kd,pid_lin.Kd);

}

void ApriltagLocation::dynamicCB(apriltag_ros::apriltag_rosConfig &config)
{
    ROS_INFO("DYNAMIC");

}

void ApriltagLocation::centerCB(const apriltag_ros::ApriltagCenter::ConstPtr &center_msg)
{
    //目标中心点在图像中的像素位置
    int center_x = center_msg->center_x;
    int center_y = center_msg->center_y;
    //整个图像的size
    int heigth = center_msg->height;
    int width = center_msg->width;
    
    std::cout<<"center: "<<center_x<<"   "<<center_y<<std::endl;    
    try
    {
        listener.waitForTransform("/tag_1","/base_link",ros::Time(0),ros::Duration(1));
        listener.lookupTransform("/tag_1","/base_link",ros::Time(0),transform);
    }
    catch(tf::TransformException &ex)   
    {
        ROS_ERROR("%S",ex.what());
        ros::Duration(1.0).sleep();
    }

    double bias = transform.getOrigin().getX() -0.2;
    double dist = transform.getOrigin().getZ();

    double R_w = transform.getRotation().getW();
    double R_x = transform.getRotation().getX();
    double R_y = transform.getRotation().getY();
    double R_z = transform.getRotation().getZ();
    tf::Quaternion quat;
    quat.setW(R_w);
    quat.setX(R_x);
    quat.setY(R_y);
    quat.setZ(R_z);
     
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    std::cout<<"bias:  "<<bias<<std::endl;
    std::cout<<"dist:  "<<dist<<std::endl;
    std::cout<<"yaw:"<<yaw<<std::endl;
    std::cout<<"roll:"<<roll<<std::endl;
    std::cout<<"pitch:"<<pitch<<std::endl;

    
    float angular;
    float x_ang,yaw_ang,x_lin;

    //小车在左，图像在左
    if ((bias>0)&&((center_x <= width / 2)))    
    {
        //目标中心点小于图像宽度的1/4,左转
        if ((center_x <= width / 4))
        {
            pid_x.SetTargetPoint(bias);
            x_ang = pid_x.Run(x_ang,1);
            angular = x_ang;
            
            vel_msg.angular.z = angular;
            vel_msg.linear.x = (dist-0.4)*0.3;        
        }
        //目标中心点大于图像宽度的1/4,右转
        else if((center_x >width / 4))
        {
            pid_x.SetTargetPoint(bias);
            x_ang = pid_x.Run(x_ang,1);
            angular = x_ang;
            
            vel_msg.angular.z = -angular;
            vel_msg.linear.x = (dist-0.4)*0.3;
        }
    }
    //小车在左，目标在右，向右转弯,使目标中心在图像的左边
    else if ((bias>0.000) && (center_x>width/2))
    {
        vel_msg.angular.z =  0.002*(width/2-center_x);
        vel_msg.linear.x = 0;
    }
    //小车在右，图像在右，向左转弯
    else if ((bias<0) && (center_x>= width/2))
    {
        //目标中心点小于图像宽度的3/4,左传
        if ((center_x<= (width / 4)*3 ))
        {
            pid_x.SetTargetPoint(bias);
            x_ang = pid_x.Run(x_ang,1);

            angular = x_ang;
            vel_msg.angular.z = -angular;
            vel_msg.linear.x = (dist-0.4) *0.3;        
        }
        //目标中心点大于图像宽度的3/4,右转
        else if((center_x >(width / 4)*3))
        {
            pid_x.SetTargetPoint(bias);
            x_ang = pid_x.Run(x_ang,1);
            angular = x_ang;

            vel_msg.angular.z = angular;
            vel_msg.linear.x = (dist-0.4)*0.3;
        }
    }
    //小车在右，图像在左，向左转弯
    else if ((bias<0.000) && (center_x< width/2))
    {
        vel_msg.angular.z = 0.002*(width/2 - center_x);
        vel_msg.linear.x = 0;
    }

    if ((bias == 0.00))
    {
        vel_msg.angular.z = -pitch;
    }
    
    if (vel_msg.linear.x>0.1)
        vel_msg.linear.x = 0.1;
    else if (vel_msg.linear.x < -0.1)
        vel_msg.linear.x = -0.1;
    
    if (vel_msg.angular.z>0.1)
        vel_msg.angular.z = 0.1;
    else if (vel_msg.angular.z < -0.1)
        vel_msg.angular.z = -0.1;
    
    std::cout<<"line::"<<vel_msg.linear.x<<std::endl;
    std::cout<<"angular::"<<vel_msg.angular.z<<std::endl;

    vel_.publish(vel_msg);
}
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "apriltag_location_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ros::Rate rate(10);
    apriltag_location::ApriltagLocation al;
    al.init(nh,nh_p);
    ros::spin();
    return 0;
}
