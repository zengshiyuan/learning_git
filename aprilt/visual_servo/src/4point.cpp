/*!
  \example tutorial-ros-pioneer-visual-servo.cpp

  Example that shows how to control the Pioneer mobile robot by IBVS visual servoing with respect to a blob.
  The current visual features that are used are s = (x, log(Z/Z*)). The desired one are s* = (x*, 0), with:
  - x the abscisse of the point corresponding to the blob center of gravity measured at each iteration,
  - x* the desired abscisse position of the point (x* = 0)
  - Z the depth of the point measured at each iteration
  - Z* the desired depth of the point equal to the initial one.

  The degrees of freedom that are controlled are (vx, wz), where wz is the rotational velocity
  and vx the translational velocity of the mobile platform at point M located at the middle
  between the two wheels.

  The feature x allows to control wy, while log(Z/Z*) allows to control vz.
  The value of x is measured thanks to a blob tracker.
  The value of Z is estimated from the surface of the blob that is proportional to the depth Z.

  */

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
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visual_servo/vpSimulatorPioneer.h>
#include <visp_ros/vpROSGrabber.h>
// #include <visp_ros/vpROSRobotPioneer.h>

#if defined(VISP_HAVE_DC1394_2) && defined(VISP_HAVE_X11)
#  define TEST_COULD_BE_ACHIEVED
#endif

#ifdef TEST_COULD_BE_ACHIEVED
void stop( );
//Sort rules 
bool op(vpDot2&a,vpDot2&b){
  
	return (a.getCog().get_i()+a.getCog().get_j())< (b.getCog().get_i()+b.getCog().get_j());
}
int main(int argc, char **argv)
{
   ros::init(argc, argv, "velocity_publisher");

    // 创建节点句柄
   ros::NodeHandle n;

  // 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
   ros::Publisher pionner_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
   geometry_msgs::Twist vel_msg;

  try {
    vpImage<unsigned char> I; // Create a gray level image container
    double depth = 0.5;
    double lambda = 0.6;
    double coef = 0.0358;// Scale parameter used to estimate the depth Z of the blob from its surface
    unsigned int j =0, k=0, l=0;
    vpSimulatorPioneer robot;
    vpCameraParameters cam;

    // Create a grabber based on libdc1394-2.x third party lib (for firewire cameras under Linux)
    vpROSGrabber g;
    g.setCameraInfoTopic("/camera_rgb/camera_info");
    g.setImageTopic("/camera_rgb/image_raw");
    g.setRectify(true);

    // Set camera parameters 
    cam.initPersProjWithoutDistortion(600,600,I.getWidth()/2, I.getHeight()/2);
    g.open(I);
    g.acquire(I);

    // Create an image viewer
    vpDisplayX c(I, 10, 10, "Current frame");
    vpDisplay::display(I);
    vpDisplay::flush(I);

    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-0.06, -0.06, 0);
    point[1].setWorldCoordinates( 0.06, -0.06, 0);
    point[2].setWorldCoordinates( 0.06,  0.06, 0);
    point[3].setWorldCoordinates(-0.06,  0.06, 0);
    


    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE) ;
    task.setLambda(lambda) ;
    vpVelocityTwistMatrix cVe ;
    cVe = robot.get_cVe() ;
    task.set_cVe(cVe) ;

    vpMatrix eJe;
    robot.get_eJe(eJe) ;
    task.set_eJe(eJe) ;

    // sets the initial camera location
  vpHomogeneousMatrix cMo(-0.3, -0.2, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(40));
  vpHomogeneousMatrix wMo; // Set to identity
  vpHomogeneousMatrix wMc; // Camera position in the world frame
  // Initialize the robot
  wMc = wMo * cMo.inverse();
    // sets the desired camera location
  vpHomogeneousMatrix cMo_d(0, 0, 1, 0, 0, 0);
  // computes the 3D point coordinates in the camera frame and its 2D
  // coordinates
  for (int i = 0; i < 4; i++)
    point[i].project(cMo_d);
  // creates the associated features
  vpFeaturePoint pd[4];
  for (int i = 0; i < 4; i++)
    vpFeatureBuilder::create(pd[i], point[i]);
  // Current visual features initialization
  // computes the 3D point coordinates in the camera frame and its 2D
  // coordinates
  for (int i = 0; i < 4; i++)
    point[i].project(cMo);
  // creates the associated features
  vpFeaturePoint p[4];
  for (int i = 0; i < 4; i++)
    vpFeatureBuilder::create(p[i], point[i]);
    // we want to see a point on a point
  for (int i = 0; i < 4; i++)
    task.addFeature(p[i], pd[i]);


    vpColVector v; // vz, wx

    while(1)
    {
      // Acquire a new image
      g.acquire(I);
      vpDisplay::display(I);
      unsigned int i =0;

      // Update the current features
      for (int i = 0; i < 4; i++) {
        point[i].project(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
      }


      robot.get_cVe(cVe) ;
      task.set_cVe(cVe) ;

      robot.get_eJe(eJe) ;
      task.set_eJe(eJe) ;

      // Compute the control law. Velocities are computed in the mobile robot reference frame
      v = task.computeControlLaw() ;

      std::cout <<"Error:"<<task.getError()<<std::endl;
      std::cout <<"Error_sum:"<<task.getError().sumSquare()<<std::endl;

      // Send the velocity to the robot
      vel_msg.linear.x =v[0];
      vel_msg.angular.z =vpMath::deg(v[1]);

        // 发布消息
      pionner_vel_pub.publish(vel_msg);
      ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
          vel_msg.linear.x, vel_msg.angular.z);
      
      vpTime::wait(0.004* 1000);
      // A click in the viewer to exit
      if ( vpDisplay::getClick(I, false) )
        break;
      if (sqrt(task.getError().sumSquare() )< 0.01) {
        std::cout << "Reached a small error. We stop the loop... " << std::endl;
        std::cout << "Ending robot thread..." << std::endl;
        vel_msg.linear.x =0;
        vel_msg.angular.z = 0;
        // 发布消息
        pionner_vel_pub.publish(vel_msg);
        std::cout << "Robot  is stopped" << std::endl;
        break;
      }
    };
    std::cout << "Ending robot thread..." << std::endl;
      vel_msg.linear.x =0;
      vel_msg.angular.z = 0;

        // 发布消息
      pionner_vel_pub.publish(vel_msg);
      std::cout << "Robot  is stopped" << std::endl;
    // Kill the servo task
    
    const char *legend = "Click to quit...";
    // vpDisplay::displayText(graph.I, (int)graph.I.getHeight() - 60, (int)graph.I.getWidth() - 150, legend, vpColor::red);
    // vpDisplay::flush(graph.I);
    // vpDisplay::getClick(graph.I);

    task.print() ;
    task.kill();
  }

  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    vel_msg.linear.x =0;
     vel_msg.angular.z = 0;

        // 发布消息
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
