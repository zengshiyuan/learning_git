
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
    vpColVector v; // 创建速度向量
  
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

    vpSimulatorPioneer robot; //创建一个pionner的仿真实例
    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);    //设置视觉伺服计算方法eyeinhand cVe_eJe
    task.setInteractionMatrixType(vpServo::CURRENT,vpServo::PSEUDO_INVERSE) ;//设置交互矩阵类型

    // double lambda = 0.3;
    vpAdaptiveGain lambda;
    lambda.initStandard(4,0.6,50);
    task.setLambda(lambda) ;//设置增益参数
    
    vpVelocityTwistMatrix cVe ;//创建cVe矩阵，描述相机与机器人之间的坐标变换关系
    cVe = robot.get_cVe() ;//从机器人中获取cVe矩阵信息
    task.set_cVe(cVe) ;//将cVe矩阵添加到视觉伺服任务中

    vpMatrix eJe;//创建eJe矩阵，机器人雅可比矩阵
    robot.get_eJe(eJe) ;//从机器人中获取eJe矩阵信息
    task.set_eJe(eJe) ;//将eJe矩阵添加到视觉伺服任务中

    //creat 3D
    // double Z_d=0.3;
    // double X =0.000,Y=0.000,Z = Z_d;
    // // vpFeatureThetaU sxz,sxzd;
    // vpFeaturePoint3D sxz,sxzd;
    // sxz.buildFrom(0,0,Z_d);
    // sxzd.buildFrom(0,0,Z_d);
    // sxz.buildFrom(X,Y,Z);
    // sxzd.buildFrom(0,0,Z_d);
    // task.addFeature(sxz,sxzd,vpFeaturePoint3D::selectX() | vpFeaturePoint3D::selectZ());


    //servo
    vpHomogeneousMatrix cdmc,cmo,omo;

    //desired pose to reach
    // vpPoseVector pose_cdmo(0.0000,0.000,0.3,0,0,M_PI);
    // vpHomogeneousMatrix cdmo(pose_cdmo);
    vpHomogeneousMatrix cdmo(vpTranslationVector(0.0, 0.0, 0.4), // 3 次标签沿相机 z 轴
                              vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );

    cdmc = cdmo * cmo.inverse();
    vpFeatureTranslation t(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
    t.buildFrom(cdmc);
    tu.buildFrom(cdmc);
    vpFeatureTranslation td(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tud(vpFeatureThetaU::cdRc);
    task.addFeature(t, td);
    task.addFeature(tu, tud);


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
          vpPoseVector pose = vpPoseVector(cMo_vec[0]);
          
          vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
          vpHomogeneousMatrix cdMo(0, 0, 0.0, 0, 0, 0);
          vpDisplay::displayFrame(I, cdMo, cam, tagSize / 2, vpColor::red, 3);
          // vpDisplay::displayFrame(I, cMo_vec[0], cam, tagSize / 2, vpColor::none, 3);

          // vpDisplay::displayCross(I, detector.getCog(0), 15, vpColor::green, 3); // Current polygon used to compure an moment
          vpImagePoint poi = detector.getCog(0);    //二维码中心点在图像中的位置（x,y)
          std::cout << "point: " << poi << std::endl;
          
          vpRect bbox = detector.getBBox(0);
          vpDisplay::displayRectangle(I,bbox,vpColor::red);
          std::vector<vpImagePoint> p = detector.getPolygon(0);
          // for (size_t i = 0; i < 4; i++)
          // {
          //     std::cout<<"point :"<<p[i]<<std::endl;
          //   vpDisplay::displayCross(I, p[i], 15, vpColor::green, 3); // Current polygon used to compure an moment
          // }

          //只有一个码
          if (cMo_vec.size() ==1)
          {
            cmo = cMo_vec[0];
            
            static bool first_time = true;
            if (first_time)
            {
              std::vector<vpHomogeneousMatrix> v_omo(2),v_cdmc(2);
              v_omo[1].buildFrom(0.0000,0.0000,0.0,0,0,M_PI);
              for (size_t i = 0; i < 2; i++)
              {
                v_cdmc[i] = cdmo * v_omo[i] * cmo.inverse();
              }
              if (std::fabs(v_cdmc[0].getThetaUVector().getTheta()) 
                  < std::fabs(v_cdmc[1].getThetaUVector().getTheta()))
              {
                omo = v_omo[0];
              }
              else{
                std::cout<<"Desired frame modified to avoid PI rotation of the camera" << std::endl;
                omo=v_omo[1];
              }              
            }
            //update visual feature
            // X=cMo_vec[0][0][3];
            // Y=cMo_vec[0][1][3];
            // Z=cMo_vec[0][2][3];
            // sxz.buildFrom(X,Y,Z);
            // std::cout << "X: " << X << " Z: " << Z << std::endl;
            cdmc = cdmo * omo *cmo.inverse();
            t.buildFrom(cdmc);
            tu.buildFrom(cdmc);
            // Display desired and current pose features
            vpDisplay::displayFrame(I, cdmo * omo, cam, tagSize / 1.5, vpColor::yellow, 2);
            vpDisplay::displayFrame(I, cmo,  cam,tagSize / 2,   vpColor::none,   3);
          }
            
            
            //更新cVe矩阵信息
            robot.get_cVe(cVe) ;
            task.set_cVe(cVe) ;
            //更新eJe矩阵信息
            robot.get_eJe(eJe) ;
            task.set_eJe(eJe) ;
            std::cout << "here" << std::endl;

            // Compute the control law. Velocities are computed in the mobile robot reference frame
            v = task.computeControlLaw();
            // // // 将速度指令发送到机器人
            vel_msg.linear.x =v[0];
            vel_msg.angular.z =v[1];
              // 发布消息
            pionner_vel_pub.publish(vel_msg);
            ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
            vel_msg.linear.x, vel_msg.angular.z);
            //读取pose的前三个数据，分别是x，y，z方向的距离，分别平方求和之后开根号得到相机坐标系与二维码坐标系的直线距离
            double distance = sqrt(pow(vpPoseVector(cMo_vec[0]).t().data[0], 2)
                      + pow(vpPoseVector(cMo_vec[0]).t().data[1], 2)
                      + pow(vpPoseVector(cMo_vec[0]).t().data[2], 2));
            std::cout << "Distance: " << distance << std::endl;
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