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
#include <visual_servo/vpSimulatorPioneer.h>//这个头文件中包含了pioneer机器人仿真参数及相关函数的声明
#include <visp_ros/vpROSGrabber.h>
// #include <visp_ros/vpROSRobotPioneer.h>

#if defined(VISP_HAVE_DC1394_2) && defined(VISP_HAVE_X11)
#  define TEST_COULD_BE_ACHIEVED
#endif

#ifdef TEST_COULD_BE_ACHIEVED

int main(int argc, char **argv)
{
   ros::init(argc, argv, "velocity_publisher");//初始化速度话题发布

    // 创建节点句柄
   ros::NodeHandle n;

  // 创建一个Publisher，发布名为/pionner/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
   ros::Publisher pionner_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
   geometry_msgs::Twist vel_msg;//创建一个geometry_msgs::Twist类型的速度消息

  try {
    vpImage<unsigned char> I; // Create a gray level image container
    double depth = 1.; //创建一个期望的距离（目标与小车之间）
    double lambda = 0.6; //视觉伺服增益参数
    double coef = 0.25;// 用于计算目标和小车之间距离的比例参数
   
    vpSimulatorPioneer robot; //创建一个pionner的仿真实例
    
    vpCameraParameters cam; //创建相机参数实例
    
    vpROSGrabber g;//创建图像捕捉实例
    g.setCameraInfoTopic("/camera_rgb/camera_info");//设置相机信息话题
    g.setImageTopic("/camera_rgb/image_raw");//设置图像话题
    g.setRectify(true);

    // Set camera parameters 
    cam.initPersProjWithoutDistortion(350,350,290, 290);//设置相机参数
    g.open(I);//打开图像捕捉实例
    g.acquire(I);//获取第一帧图像

    //创建图像显示器，并显示图像
    vpDisplayX d(I, 10, 10, "Current frame");
    vpDisplay::display(I);
    vpDisplay::flush(I);

    // 创建一个连通区域跟踪器，根据鼠标点击来选择连通区域
    vpDot2 dot;
    dot.setGraphics(true);
    dot.setComputeMoments(true);
    dot.setEllipsoidShapePrecision(0.);  // to track a blob without any constraint on the shape
    dot.setGrayLevelPrecision(0.9);  // to set the blob gray level bounds for binarisation
    dot.setEllipsoidBadPointsPercentage(0.5); // to be accept 50% of bad inner and outside points with bad gray level
    dot.initTracking(I);//跟踪连通区域
    //输出连通区域中心的坐标
    std::cout << "dot_cog_x: " <<  dot.getCog().get_u()
                        << "dot_cog_y: " <<  dot.getCog().get_v()
                        << std::endl;
    std::cout <<"dot_size:"<<dot.m00<<std::endl ;//输出连通区域的一阶图像距，可以表示图像的面积信息
    vpDisplay::flush(I);
    
    //创建一个视觉伺服人物
    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;//设置视觉伺服计算方法eyeinhand cVe_eJe
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;//设置交互矩阵类型
    task.setLambda(lambda) ;//设置增益参数
    
    vpVelocityTwistMatrix cVe ;//创建cVe矩阵，描述相机与机器人之间的坐标变换关系
    cVe = robot.get_cVe() ;//从机器人中获取cVe矩阵信息
    task.set_cVe(cVe) ;//将cVe矩阵添加到视觉伺服任务中
    std::cout << "cVe: \n" << cVe << std::endl;//打印cVe矩阵参数

    vpMatrix eJe;//创建eJe矩阵，机器人雅可比矩阵
    robot.get_eJe(eJe) ;//从机器人中获取eJe矩阵信息
    task.set_eJe(eJe) ;//将eJe矩阵添加到视觉伺服任务中
    std::cout << "eJe: \n" << eJe << std::endl;//打印eJe矩阵参数

    // 创建目标中心点的x坐标特征，s_x表示当前位置，s_xd表示期望位置
    vpFeaturePoint s_x, s_xd;

    // 根据相机参数和连通区域中心点的坐标获取当前位置下的x坐标特征
    vpFeatureBuilder::create(s_x, cam, dot);
   
    // 创建期望位置处的x坐标特征，0.5表示x方向是图像中心，0.89是y方向对应的图像位置，depth是空间坐标系下的距离
    s_xd.buildFrom(0.5, 0.89, depth);

    // 把x坐标特征添加到视觉伺服任务中
    task.addFeature(s_x, s_xd) ;

    // 创建目标中心点的z坐标特征log(Z/Zd)，s_Z表示当前位置，s_Zd表示期望位置
    vpFeatureDepth s_Z, s_Zd;

    //根据连通区域的面积来估计相机与目标之间的距离，距离的平方与面积成反比例关系，面积约小表示距离越远（近大远小）
    double surface = 1./sqrt(dot.m00/(cam.get_px()*cam.get_py()));
    double Z, Zd;//创建目标中心点的空间距离参数，Z表示当前位置，Zd表示期望位置
    Z = coef * surface ;      //初始化目标与相机之间的距离
    Zd = Z;// 设置期望距离等于当前距离
    std::cout << "Z :" << Z << std::endl; //输出当前位置处的距离信息
    s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z , 0); // 设置当前位置处的Z坐标特征，因为Z=Zd，所以特征log(Z/Zd)=0
    s_Zd.buildFrom(s_x.get_x(), s_x.get_y(), Zd , 0); // 设置期望位置处的Z坐标特征，因为Z=Zd，所以特征log(Z/Zd)=0
    task.addFeature(s_Z, s_Zd) ;//把Z坐标特征添加到视觉伺服任务中

    vpColVector v; // 创建速度向量

    while(1)
    {
      g.acquire(I);//获取相机图像
      vpDisplay::display(I);//显示相机图像内容
      dot.track(I);//跟踪目标连通区域
      
      vpFeatureBuilder::create(s_x, cam, dot);//更新x坐标特征
      // 更新Z坐标特征
      surface = 1./sqrt(dot.m00/(cam.get_px()*cam.get_py()));
      Z = coef * surface ;
      s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd)) ;

     //更新cVe矩阵信息
      robot.get_cVe(cVe) ;
      task.set_cVe(cVe) ;
     //更新eJe矩阵信息
      robot.get_eJe(eJe) ;
      task.set_eJe(eJe) ;

      // 根据视觉伺服算法计算控制律
      v = task.computeControlLaw() ;
      //输出速度信息
      std::cout << "Send velocity to the pionner: " << v[0] << " m/s "
                << vpMath::deg(v[1]) << " deg/s" << std::endl;

      // 将速度指令发送到机器人
      vel_msg.linear.x =v[0];
      vel_msg.angular.z =vpMath::deg(v[1]);

        // 发布消息
      pionner_vel_pub.publish(vel_msg);
      ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
          vel_msg.linear.x, vel_msg.angular.z);
          
      std::cout <<"s_x:"<<s_x.get_x()<<std::endl;
      std::cout <<"s_y:"<<s_x.get_y()<<std::endl ;
      std::cout << "dot_cog_x: " <<  dot.getCog().get_u()
                        << "dot_cog_y: " <<  dot.getCog().get_v()
                        << std::endl;
      std::cout <<"Z:"<<Z<<std::endl ;
      std::cout <<"Error:"<<task.getError()[0]<<std::endl;//输出x方向上的误差信息
      
      // 绘制一条绿色的竖线表示x方向上的期望位置
      vpDisplay::displayLine(I, 0, 300, 600, 300, vpColor::green);
      vpDisplay::flush(I);

      // 点击图像退出
      if ( vpDisplay::getClick(I, false) )
        break;
      //当x方向的误差小于阈值时，命令机器人停止运动
      if (abs(task.getError()[0])< 0.001) {
        std::cout << "Reached a small error. We stop the loop... " << std::endl;
        std::cout << "Ending robot thread..." << std::endl;
        vel_msg.linear.x =0;
        vel_msg.angular.z = 0;
        pionner_vel_pub.publish(vel_msg);
        std::cout << "Robot  is stopped" << std::endl;
      }
    };
    //程序结束后，命令小车停止运动
    std::cout << "Ending robot thread..." << std::endl;
    vel_msg.linear.x =0;
    vel_msg.angular.z = 0;
    pionner_vel_pub.publish(vel_msg);
    std::cout << "Robot  is stopped" << std::endl;

    task.print() ;//输出视觉伺服控制信息
    task.kill();//结束视觉伺服控制
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    
    //有异常抛出时，命令机器人停止运动
    vel_msg.linear.x =0;
    vel_msg.angular.z = 0;
    pionner_vel_pub.publish(vel_msg);
    std::cout << "Robot  is stopped" << std::endl;
    return 1;
  }
}
#else
int main()
{
  std::cout << "You don't have the right 3rd party libraries to run this example..." << std::endl;
}
#endif

