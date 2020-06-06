#include <iostream>
#include <console_teleop/console_teleop.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    std::string publishTopic = "/joy";
    ros::init(argc, argv, "avocet_console_op");
    ros::NodeHandle nh("~");
    nh.getParam("publish_topic", publishTopic);
    avc::TeleOperatorNode ton(nh, publishTopic);

    ROS_INFO_STREAM("TELEOP NODE publish topic = " << publishTopic);
    avc::BiWheelCommand bwc;
    while (nh.ok()) {
        std::string buf;
        std::cout << ">" << std::endl;
        while (std::cin >> buf)
        {
            if (buf == "q")
            {
                bwc.leftWheel = avc::SimpleTireOperation::STOP;
                bwc.rightWheel = avc::SimpleTireOperation::STOP;
                ton.setCommand(bwc);
                ton.publishJoy();
                return 0;
            }

            if (buf == "f")
            {
                bwc.leftWheel = avc::SimpleTireOperation::FORWARD;
                ton.setCommand(bwc);
                ton.publishJoy();
                break;
            }
            if (buf == "d")
            {
                bwc.leftWheel = avc::SimpleTireOperation::BACKWARD;
                ton.setCommand(bwc);
                ton.publishJoy();
                break;
            }
            if (buf == "j")
            {
                bwc.rightWheel = avc::SimpleTireOperation::FORWARD;
                ton.setCommand(bwc);
                ton.publishJoy();
                break;
            }
            if (buf == "k")
            {
                bwc.rightWheel = avc::SimpleTireOperation::BACKWARD;
                ton.setCommand(bwc);
                ton.publishJoy();
                break;
            }

            bwc.leftWheel = avc::SimpleTireOperation::STOP;
            bwc.rightWheel = avc::SimpleTireOperation::STOP;
            ton.setCommand(bwc);
            ton.publishJoy();

            std::cout << ">" << std::endl;
        }
    }
}
