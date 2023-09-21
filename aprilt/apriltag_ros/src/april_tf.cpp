#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "apriltag_ros/ApriltagCenter.h"



class PID
{
  public:
    PID() = default;
    PID(float Kp, float Ki, float Kd): Kp(Kp), Ki(Ki), Kd(Kd) {}
    ~PID() = default;
    void SetTargetPoint(float Point)
    {
      SetPoint = Point;
    }
    float Run(float Input, float DeltaTime)
    {
      float Error = SetPoint - Input;

      Proportional = Kp * Error;
      Integral += Ki * Error * DeltaTime;
      // Avoid integral windup
    //   Integral = std::clamp(Integral, MinOutput, MaxOutput);
      Derivative = (-Kd * (Input - LastInput)) / DeltaTime;

      float Out = Proportional + Integral + Derivative;
    //   Out = std::clamp(Out, MinOutput, MaxOutput);

      // Keep track of the state
      LastError = Out;
      LastInput = Input;

      return Out;
    }

  void Reset()
  {
    Proportional = 0.0f;
    Integral = 0.0f;
    // Integral = std::clamp(Integral, MinOutput, MaxOutput);
    Derivative = 0.0f;

    LastError = 0.0f;
    LastInput = 0.0f;
  }

  public:
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;

  private:
    float SetPoint;

    // Out Limits
    float MinOutput = -1.0f;
    float MaxOutput = 1.0f;

    // Internal state.
    float Proportional = 0.0f;
    float Integral = 0.0f;
    float Derivative = 0.0f;

    float LastError = 0.0f;
    float LastInput = 0.0f;
};


ros::Publisher vel_;
void CenterCB(const apriltag_ros::ApriltagCenter::ConstPtr & center_msg)
{
    PID pid_ang,pid_lin;
    PID pid_x,pid_yaw;
    pid_ang.Kp = 0.15;
    pid_ang.Ki = 0;
    pid_ang.Kd = 0;

    pid_lin.Kp = 2;
    pid_lin.Ki = 0.0;
    pid_lin.Kd = 0.0;

    pid_x.Kp = 0.3;
    pid_x.Ki = 0.0001;
    pid_x.Kd = 0.0001;
    
    pid_yaw.Kp = 0.1;
    pid_yaw.Ki = 0.0001; 
    pid_yaw.Kd = 0.0001;

    int center_x = center_msg->center_x;
    int center_y = center_msg->center_y;
    std::cout<<"center: "<<center_x<<"   "<<center_y<<std::endl;
    
    geometry_msgs::Twist vel_msg;
    tf::TransformListener listener;
    tf::StampedTransform  transform;    //建立变换矩阵
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
    double w = transform.getRotation().getW();
    double yaw = tf::getYaw(transform.getRotation());

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

    std::cout<<"x:  "<<bias<<std::endl;
    std::cout<<"z:  "<<dist<<std::endl;
    std::cout<<"w:  "<<w<<std::endl;
    std::cout<<"yaw:"<<yaw<<std::endl;
    
    float angular;
    float L;   //
    L = 0.7 * bias + 0.3 * (-w);

    float x_ang,yaw_ang;

    //小车在左，图像在左，向左转弯
    if ((bias>0)&&((center_x<=640)))
    {
        if ((center_x<=310))
        {
            pid_x.SetTargetPoint(bias);
            x_ang = pid_x.Run(x_ang,1);
            // pid_yaw.SetTargetPoint((-w));
            // yaw_ang = pid_yaw.Run(yaw_ang,1);
            std::cout<<"x_ang:"<<x_ang<<std::endl;
            // std::cout<<"yaw_ang:"<<yaw_ang<<std::endl;
            // angular = 0.5*x_ang+ 0.5* yaw_ang;
            angular = x_ang*1.2;
            vel_msg.angular.z = angular;
            vel_msg.linear.x = (dist-0.4)*0.3;        
        }
        else if((center_x >310))
        {
            pid_x.SetTargetPoint(bias);
            x_ang = pid_x.Run(x_ang,1);
            // pid_yaw.SetTargetPoint((-w));
            // yaw_ang = pid_yaw.Run(yaw_ang,1);
            // std::cout<<"x_ang:"<<x_ang<<std::endl;
            // std::cout<<"yaw_ang:"<<yaw_ang<<std::endl;
            // angular = 0.5*x_ang+ 0.5* yaw_ang;
            angular = x_ang*0.8;
            
            vel_msg.angular.z = -angular;
            vel_msg.linear.x = (dist-0.4)*0.3;
        }
    }
    //小车在左，图像在右，向右转弯
    else if ((bias>0.000) && (center_x>640))
    {
        vel_msg.angular.z =  0.002*(640-center_x);
        vel_msg.linear.x = 0;
    }
    //小车在右，图像在右，向左转弯
    else if ((bias<0) && (center_x>=640))
    {
        if ((center_x<=970))
        {
            pid_x.SetTargetPoint(bias);
            x_ang = pid_x.Run(x_ang,1);
            // pid_yaw.SetTargetPoint((-w));
            // yaw_ang = pid_yaw.Run(yaw_ang,1);
            // std::cout<<"x_ang:"<<x_ang<<std::endl;
            // std::cout<<"yaw_ang:"<<yaw_ang<<std::endl;
            // angular = 0.5*x_ang+ 0.5* yaw_ang;

            angular = x_ang*0.8;
            vel_msg.angular.z = -angular;
            vel_msg.linear.x = (dist-0.4) *0.3;        
        }
        else if((center_x >970))
        {
            pid_x.SetTargetPoint(bias);
            x_ang = pid_x.Run(x_ang,1);
            // pid_yaw.SetTargetPoint((-w));
            // yaw_ang = pid_yaw.Run(yaw_ang,1);
            // std::cout<<"x_ang:"<<x_ang<<std::endl;
            // std::cout<<"yaw_ang:"<<yaw_ang<<std::endl;
        
            // angular = 0.5*x_ang+ 0.5* yaw_ang;
            angular = x_ang*1.2;

            vel_msg.angular.z = angular;
            vel_msg.linear.x = (dist-0.4)*0.3;
        }
    }
    //小车在右，图像在左，向左转弯
    else if ((bias<0.000) && (center_x<640))
    {
        vel_msg.angular.z = 0.002*(640-center_x);
        vel_msg.linear.x = 0;
    }

    if ((bias == 0.00))
    {
        vel_msg.angular.z = -w;
    }
    

    // else
    // {
    //     pid_x.SetTargetPoint(x);
    //     x_ang = pid_x.Run(x_ang,1);
    //     pid_yaw.SetTargetPoint((-w));
    //     yaw_ang = pid_yaw.Run(yaw_ang,1);
    //     std::cout<<"x_ang:"<<x_ang<<std::endl;
    //     std::cout<<"yaw_ang:"<<yaw_ang<<std::endl;
        
    //     angular = 0.8*x_ang+ 0.2* yaw_ang;
    //     vel_msg.angular.z = angular;
    //     vel_msg.linear.x = (z-0.2) * 0.25;
    // }
    
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

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tf_listent_node");
    ros ::NodeHandle nh;
    
    vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::Subscriber tag_center_sub_  = nh.subscribe<apriltag_ros::ApriltagCenter>("/apriltag_ros_continuous_node/tag_center",1,&CenterCB);
    ros::Rate rate(30);
    ros::spin();
    return 0;
}
