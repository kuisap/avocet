#pragma once

#include <string>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace avc
{
  enum class SimpleTireOperation{
    FORWARD, STOP, BACKWARD
  };

  struct BiWheelCommand
  {
    SimpleTireOperation leftWheel;
    SimpleTireOperation rightWheel;
  };

  class TeleOperatorNode
  {
    public:
      TeleOperatorNode(ros::NodeHandle & nh, const std::string& topicName);
      ~TeleOperatorNode();
      void setCommand(const BiWheelCommand & command);
      void publishJoy();

    private:
      sensor_msgs::Joy joy_;
      ros::Publisher pub_;

  };
}
