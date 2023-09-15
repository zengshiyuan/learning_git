#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "apriltag_ros/ApriltagCenter.h"

int pid_linear(double target_linear, double current_linear);
int pid_angular(double target_angular, double current_angular);


ros::Publisher vel_;
void CenterCB(const apriltag_ros::ApriltagCenter::ConstPtr & center_msg)
{
    int center_x = center_msg->center_x;
    int center_y = center_msg->center_y;
    std::cout<<"center: "<<center_x<<"   "<<center_y<<std::endl;
    
    geometry_msgs::Twist vel_msg;
    tf::TransformListener listener;
    tf::StampedTransform  transform;    //建立变换矩阵
    try
    {
    //     listener.waitForTransform("/base_link","/apriltag",ros::Time(0),ros::Duration(1));
    //     listener.lookupTransform("/base_link","/apriltag",ros::Time(0),transform);
        listener.waitForTransform("/tag_1","/camera_link",ros::Time(0),ros::Duration(1));
        listener.lookupTransform("/tag_1","/camera_link",ros::Time(0),transform);
    }
    catch(tf::TransformException &ex)   
    {
        ROS_ERROR("%S",ex.what());
        ros::Duration(1.0).sleep();
    }

    double x = transform.getOrigin().getX();
    double z = transform.getOrigin().getZ();
    double w = transform.getRotation().getW();
    std::cout<<"x:  "<<x<<std::endl;
    std::cout<<"z:  "<<z<<std::endl;
    std::cout<<"yaw:"<<w<<std::endl;
    
    //判断中心点是否在100-500范围内
    double angular = 0;
    if (((x > 0) && (center_x>300)) || ((x<0)&& (center_x < 300)))
                {
                    vel_msg.angular.z = (300-center_x) * 0.001;
                    vel_msg.linear.x = 0;
                }
    if ((center_x> 100) && (center_y < 500))
    {
        if (z > 0.2)
        {
            if ((x!=0.00) && (z>0.30))
            {
                if(((x<0)&& (center_x >= 300)) || ((x>0) && (center_x <= 300)))
                {
                    vel_msg.linear.x = fabs(x) * 3;
                    vel_msg.angular.z = (300-center_x) * 0.0006;
                }
            }      
            else if((x!= 0.00) && ((z<=0.30)))
            {
                vel_msg.linear.x = fabs(x) * 5;
                vel_msg.angular.z = -(center_x - 300) * 0.0005;
            }
            else if ((x == 0.00))
            {
                if (w!=0.00)
                    vel_msg.angular.z = - w ;
                else
                    vel_msg.linear.x = (z-0.2) * 0.2;
            }

            else if ((x == 0.00))
            {
                if (w!=0.00)
                    vel_msg.angular.z = - w ;
                else
                    vel_msg.linear.x = (z-0.3) * 0.1;
            }
        }
        else
        {
            vel_msg.linear.x = (z-0.2) * 0.1;
            if (w !=0.000)
            {
                vel_msg.angular.z = -w;
            }
            // if ((center_x > 310) || (center_x <290))
            // {
            //     vel_msg.linear.x = -0.05;
            //     vel_msg.angular.z = (center_x -300) * 0.0005;
            //     ros::Duration(3);
            // }
            // else
            // {
            //     vel_msg.linear.x = (z-0.2) * 0.2;
            //     vel_msg.angular.z = -w*2;;
            // }
        }
    }
    else {
        if (center_x < 100)
        {
            vel_msg.angular.z = 0.3;
            vel_msg.linear.x = 0;
        }
        if (center_x > 500)
        {
            vel_msg.angular.z = -0.2;
            vel_msg.linear.x = 0;
        }
    }

    // 限制最大最小速度
    if (vel_msg.linear.x > 0.1)
        vel_msg.linear.x =0.1;
    else if (vel_msg.linear.x < -0.1)
        vel_msg.linear.x = -0.1;
    else
        vel_msg.linear.x = vel_msg.linear.x;
    
    // //限制角速度
    // if (vel_msg.angular.z > 0.1)
    //     vel_msg.angular.z = 0.1;
    // else if (vel_msg.angular.z < -0.1)
    //     vel_msg.angular.z = -0.1;
    // else
    //     vel_msg.angular.z = vel_msg.angular.z;

    std::cout<<"line::"<<vel_msg.linear.x<<std::endl;
    std::cout<<"angular::"<<vel_msg.angular.z<<std::endl;

    vel_.publish(vel_msg);

}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tf_listent_node");
    ros ::NodeHandle nh;

    vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",5);
    ros::Subscriber tag_center_sub_  = nh.subscribe<apriltag_ros::ApriltagCenter>("/apriltag_ros_continuous_node/tag_center",1,&CenterCB);
    ros::Rate rate(10);
    
    // while (ros::ok())
    // {
    //     tf::TransformListener listener;
    //     tf::StampedTransform  transform;    //建立变换矩阵
    //     try
    //     {
    //         // listener.waitForTransform("/apriltag","/base_link",ros::Time(0),ros::Duration(1));
    //         // listener.lookupTransform("/apriltag","/base_link",ros::Time(0),transform);
    //         listener.waitForTransform("/tag_1","/camera_link",ros::Time(0),ros::Duration(1));
    //         listener.lookupTransform("/tag_1","/camera_link",ros::Time(0),transform);
    //     }
    //     catch(tf::TransformException &ex)   
    //     {
    //         ROS_ERROR("%S",ex.what());
    //         ros::Duration(1.0).sleep();
    //     }

    //     double x = transform.getOrigin().getX();
    //     double y = transform.getOrigin().getY();
    //     double z = transform.getOrigin().getZ();
    //     double w = transform.getRotation().getW();
    //     std::cout<<"x:  "<<x<<std::endl;
    //     std::cout<<"y:  "<<y<<std::endl;
    //     std::cout<<"z:  "<<z<<std::endl;
    //     std::cout<<"yaw:"<<w<<std::endl;
    //     rate.sleep();
    // }
        
    ros::spin();
    return 0;
}

int pid_linear(double target_linear, double current_linear)
{
    double kp = 1,ki=0.0,kd=0.0;
    double err,err_last,err_last_last;

    err = target_linear -current_linear;
    double increment = kp*(err - err_last) + ki*err +
                       kd*(err - 2*err_last + err_last_last);
    current_linear += increment;
    err_last_last = err_last;
    err_last = err;
    return current_linear;
}
int pid_angular(double target_angular, double current_angular)
{
    double kp = 0.007,ki=0.0,kd=0.0;
    double err,err_last,err_last_last;

    err = target_angular -current_angular;
    double increment = kp*(err - err_last) + ki*err +
                       kd*(err - 2*err_last + err_last_last);
    current_angular += increment;
    err_last_last = err_last;
    err_last = err;
    return current_angular;
}
