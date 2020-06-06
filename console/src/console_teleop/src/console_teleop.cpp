#include "console_teleop/console_teleop.h"
#include <vector>


namespace avc
{

  TeleOperatorNode::TeleOperatorNode(ros::NodeHandle & nh, const std::string& topicName = "/joy") : pub_(nh.advertise<sensor_msgs::Joy>(topicName, 1)) {}

  TeleOperatorNode::~TeleOperatorNode() {}


  void TeleOperatorNode::setCommand(const BiWheelCommand &command)
  {
    sensor_msgs::Joy joy;
    joy.buttons = std::vector<int>(8);
    joy.buttons[4] = joy.buttons[5] = joy.buttons[6] = joy.buttons[7] = 0;

    {
      switch(command.leftWheel) {
        case SimpleTireOperation::FORWARD:
          joy.buttons[4] = 1;
          break;
        case SimpleTireOperation::BACKWARD:
          joy.buttons[6] = 1;
          break;
      }

      switch(command.rightWheel) {
        case SimpleTireOperation::FORWARD:
          joy.buttons[5] = 1;
          break;
        case SimpleTireOperation::BACKWARD:
          joy.buttons[7] = 1;
          break;
      }
    }
    joy_ = std::move(joy);
  }

  void TeleOperatorNode::publishJoy()
  {
    pub_.publish(joy_);
  }

}
