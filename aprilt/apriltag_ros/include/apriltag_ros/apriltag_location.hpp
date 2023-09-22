#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "apriltag_ros/ApriltagCenter.h"
#include <dynamic_reconfigure/server.h>
#include "apriltag_ros/apriltag_rosConfig.h"

namespace apriltag_location{
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
      float Kp;
      float Ki;
      float Kd;

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

  class ApriltagLocation
  {
  private:
      
      

  public:
      ApriltagLocation();

      void init(ros::NodeHandle nh,ros::NodeHandle nh_p);

      void centerCB(const apriltag_ros::ApriltagCenter::ConstPtr &center_msg);
      void dynamicCB(apriltag_ros::apriltag_rosConfig &config);

      PID pid_ang,pid_lin;
      PID pid_x,pid_yaw;

      geometry_msgs::Twist vel_msg;
      tf::TransformListener listener;
      tf::StampedTransform  transform;    //建立变换矩阵
      ros::Publisher vel_;
      ros::Subscriber tag_center_sub_;

  };

}
