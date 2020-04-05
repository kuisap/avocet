#include <ros/ros.h>
#include <wiringPi.h>
#include "driver_controllers.h"
#include <sensor_msgs/Joy.h>

class MotorDriveNode
{
public:
  MotorDriveNode()
  {

    int vcc = 21, a1 = 27, a2 = 22, b1 = 23, b2 = 24;
    nh_.getParam("vcc", vcc);
    nh_.getParam("a1", a1);
    nh_.getParam("a2", a2);
    nh_.getParam("b1", b1);
    nh_.getParam("b2", b2);
    controller = avc::MotorController(vcc, a1, a2, b1, b2);
    controller.turnOn();
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &MotorDriveNode::joyCallback, this);
  }

  ~MotorDriveNode() {
    controller.turnOff();
  }

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
    const auto L1 = joy->buttons[4];
    const auto L2 = joy->buttons[6];
    const auto R1 = joy->buttons[5];
    const auto R2 = joy->buttons[7];

    if (L1 ==0 && L2 == 0){ controller.stop(0); }
    else if (L1 == 1 && L2 == 0){ controller.forward(0);}
    else if (L1 == 0 && L2 == 1){ controller.backward(0);}


    if (R1 ==0 && R2 == 0){ controller.stop(1); }
    else if (R1 == 1 && R2 == 0){ controller.forward(1);}
    else if (R1 == 0 && R2 == 1){ controller.backward(1);}

    ROS_INFO_STREAM("Button L1 = " << L1 << ", R1 = " << R1 << ", L2 = " << L2 << ", R2 = " << R2);

  }
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  avc::MotorController controller;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "driver_controller");
  ROS_INFO_STREAM("Driver Controller is enabled.");
  MotorDriveNode motorDriveNode;
  ros::spin();
  return 0;
}