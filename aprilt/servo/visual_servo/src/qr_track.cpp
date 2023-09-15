
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpServo.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visual_servo/vpSimulatorPioneer.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotPioneer.h>

#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpPose.h>

#include <stdlib.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp/vpV4l2Grabber.h>

#include <cmath>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"qr_track_node");
    ros::NodeHandle nh;

    ros::Publisher pionner_vel_pub= nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    geometry_msgs::Twist vel_msg;
// #if defined(VISP_HAVE_ZBAR)
#if (defined(VISP_HAVE_ZBAR) || defined(VISP_HAVE_DMTX)) &&                                                            \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
    int opt_device = 0;             // For OpenCV and V4l2 grabber to set the camera device
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;  
  double tagSize = 0.12;
  float quad_decimate = 1.0;
  int nThreads = 1;
  std::string intrinsic_file = "";
  std::string camera_name = "";
  bool display_tag = false;
  int color_id = -1;
  unsigned int thickness = 2;
  bool align_frame = false;
  bool flag =true;
  #if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  bool display_off = true;
  std::cout << "Warning: There is no 3rd party (X11, GDI or openCV) to dislay images..." << std::endl;
#else
  bool display_off = false;
#endif
  vpImage<unsigned char> I;
    try
    {
    vpCameraParameters cam; //创建相机参数实例
    vpROSGrabber g;//创建图像捕捉实例
    g.setCameraInfoTopic("/camera_rgb/camera_info");//设置相机信息话题
    g.setImageTopic("/camera_rgb/image_raw");//设置图像话题
    g.setRectify(true);

    // Set camera parameters 
    cam.initPersProjWithoutDistortion(357.53,357.53,300.5, 300.5);//设置相机参数
    vpXmlParserCamera parser;
    if (!intrinsic_file.empty() && !camera_name.empty())
      parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
    g.open(I);//打开图像捕捉实例
    //创建图像显示器，并显示图像
    g.acquire(I);//获取相机图像
    vpDisplay::flush(I);

    vpDisplay *d = NULL;
    if (! display_off) {
#ifdef VISP_HAVE_X11
      d = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
      d = new vpDisplayGDI(I);
#elif defined(VISP_HAVE_OPENCV)
      d = new vpDisplayOpenCV(I);
#endif
    }
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
    // detector.setZAlignedWithCameraAxis(align_frame);

    std::vector<double> time_vec;
    for (;;) {
        g.acquire(I);
        vpDisplay::display(I);//显示相机图像内容
        
        double t_ = vpTime::measureTimeMs();
        std::vector<vpHomogeneousMatrix> cMo_vec;
        if(detector.detect(I, tagSize, cam, cMo_vec))
        {
          t_ = vpTime::measureTimeMs() - t_;
          time_vec.push_back(t_);
          std::stringstream ss;
          ss << "Detection time: " << t_ << " ms for " << detector.getNbObjects() << " tags";
          // vpHomogeneousMatrix cmo;
          std::cout<<"pose: "<<vpPoseVector(cMo_vec[0]).t()<<std::endl;

          vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
          vpHomogeneousMatrix cdMo(0, 0, 0.0, 0, 0, 0);
          // vpDisplay::displayFrame(I, cdMo, cam, tagSize / 2, vpColor::red, 3);
          // vpDisplay::displayFrame(I, cMo_vec[0], cam, tagSize / 2, vpColor::none, 3);

          vpDisplay::displayCross(I, detector.getCog(0), 15, vpColor::red, 3); // Current polygon used to compure an moment
          vpImagePoint center_point = detector.getCog(0);    //二维码中心点在图像中的位置（x,y)
          std::cout << "point: " << center_point << std::endl;
          vpRect bbox = detector.getBBox(0);
          vpDisplay::displayRectangle(I,bbox,vpColor::red);
          std::vector<vpImagePoint> p = detector.getPolygon(0);
          for (size_t i = 0; i < 4; i++)
          {
              // std::cout<<"point :"<<p[i]<<std::endl;
            vpDisplay::displayCross(I, p[i], 15, vpColor::green, 3); // Current polygon used to compure an moment
          }
          // vpDisplay::displayFrame(I, cMo_vec[0], cam, tagSize / 0.5, vpColor::yellow, 2);
          //读取pose的前三个数据，分别是x，y，z方向的距离，分别平方求和之后开根号得到相机坐标系与二维码坐标系的直线距离
          double distance = sqrt(pow(vpPoseVector(cMo_vec[0]).t().data[0], 2)
                    + pow(vpPoseVector(cMo_vec[0]).t().data[1], 2)
                    + pow(vpPoseVector(cMo_vec[0]).t().data[2], 2));
          double x = cMo_vec[0][0][3];
          double y = cMo_vec[0][1][3];
          double z = cMo_vec[0][2][3];
          double current_yaw = vpPoseVector(cMo_vec[0]).t().data[4];

          std::cout << "Distance: " << distance << std::endl;
          double targat_sin = 0.00;  //目标夹角的sin值
          double target_yaw = M_PI;
          double target_distance = 0.4;

          double current_sin = x / distance; //求x和dist的sin值
          std::cout << "current_sin: " << current_sin << std::endl;

          double dist_error = distance - target_distance;
          double yaw_error;
          double sin_error = -(current_sin - targat_sin);

          double angurlar_ ;

          if (current_yaw > 0)
          {
            yaw_error = target_yaw - current_yaw;
          }
          else
            yaw_error = -(target_yaw + current_yaw);

          std::cout << "yaw_error: " << yaw_error << std::endl;
          std::cout << "sin_error: " << sin_error << std::endl;
          
          double vel_ = dist_error * 0.1;
          angurlar_ = sin_error  + yaw_error ;

          if (angurlar_ > 0.4 )
          {
            angurlar_ = 0.4;
          }
          else if (angurlar_ < -0.4)
          {
            angurlar_ = -0.4;
          }
          else
            angurlar_ = angurlar_;
          
          // // // 将速度指令发送到机器人
          vel_msg.linear.x =vel_;
          vel_msg.angular.z =angurlar_;
            // 发布消息
          pionner_vel_pub.publish(vel_msg);
          ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
          vel_msg.linear.x, vel_msg.angular.z);
          
        }
        else{
          vpDisplay::displayText(I, 40, 20, "Missing Apriltag", vpColor::red);

        }
      vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }
    if (! display_off)
      delete d;
    }
    catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
    
return EXIT_SUCCESS;

#endif
  return EXIT_SUCCESS;
}