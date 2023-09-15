
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
  
  double tagSize = 0.14;
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
    cam.initPersProjWithoutDistortion(357,357,300, 300);//设置相机参数
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
    detector.setZAlignedWithCameraAxis(align_frame);
    
    vpHomogeneousMatrix cdmo(0,0,0.5,0,0,0);
    std::vector<vpPoint> point;//目标物体特征点的空间坐标容器
    point.push_back(vpPoint(-0.06, -0.06, 0));
    point.push_back(vpPoint(0.06, -0.06, 0));
    point.push_back(vpPoint(0.06, 0.06, 0));
    point.push_back(vpPoint(-0.06, 0.06, 0));

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
          vpHomogeneousMatrix cmo;
          // detector.getPose(0,tagSize,cam,cmo);
          // vpPoseVector pose(cmo);
          
          std::cout << "pose: " << vpPoseVector(cMo_vec[0]).t()  << std::endl;
          
          vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
          vpDisplay::displayCross(I, detector.getCog(0), 15, vpColor::green, 3); // Current polygon used to compure an moment
          vpImagePoint point = detector.getCog(0);    //二维码中心点在图像中的位置（x,y)
          std::cout << "point: " << point << std::endl;

            vpDisplay::displayFrame(I, cMo_vec[0], cam, tagSize / 2, vpColor::none, 3);
            vpRotationMatrix R = cMo_vec[0].getRotationMatrix();
          // std::cout << "R: " << R << std::endl;
            double current_yaw = atan2(R[1][0], R[0][0]);
          // std::cout << "current_yaw: " << current_yaw << std::endl;

          
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