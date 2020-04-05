#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>

int main(int argc, char** argv)
{
  int width = 300, height = 300;
  float fps = 30;
  std::string publishTopic = "camera/image";


  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh("~");
  nh.getParam("width", width);
  nh.getParam("height", height);
  nh.getParam("fps", fps);
  nh.getParam("publish_topic", publishTopic);


  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(publishTopic, 1);
  cv::VideoCapture cap(0);
  if(!cap.isOpened()){
    ROS_ERROR_STREAM("Camera could not be opened.");
    return 1;
  }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap.set(cv::CAP_PROP_FPS, fps);

  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(fps);
  ROS_INFO_STREAM("WIDTH = " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << ", HEIGHT = " <<  cap.get(cv::CAP_PROP_FRAME_HEIGHT) << ", FPS = " << cap.get(cv::CAP_PROP_FPS) << ", publish topic = " << publishTopic);
  while (nh.ok()) {
    cap >> frame;
    if(!frame.empty()) {
      auto header = std_msgs::Header();
      header.stamp = ros::Time::now();
      ROS_INFO_STREAM("Time " << header.stamp);
      auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      pub.publish(std::move(msg));
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
