#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  float fps = 100;
  std::string publishTopic = "/imu";
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle nh("~");
  nh.getParam("fps", fps);
  nh.getParam("publish_topic", publishTopic);

  sensor_msgs::ImuPtr msg;

  ros::Rate loop_rate(fps);
  ROS_INFO_STREAM("IMU fps = " << fps << ", publish topic = " << publishTopic);
  while (nh.ok()) {
    //   cap >> frame;
    //   if(!frame.empty()) {
    //     auto header = std_msgs::Header();
    //     header.stamp = ros::Time::now();
    //     ROS_INFO_STREAM("Time " << header.stamp);
    //     auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    //     pub.publish(std::move(msg));
    //   }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
