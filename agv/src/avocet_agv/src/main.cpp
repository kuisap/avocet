#include <image_capture.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

int main(int argc, char** argv) {
  int width = 300, height = 300;
  float fps = 30;
  std::string publishTopic = "camera/image";
  std::string config_str = "./src/avocet_agv/config/config.yaml";

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh("~");
  nh.getParam("config", config_str);

  YAML::Node config = YAML::LoadFile(config_str);
  if (config["image"]["width"]) {
    width = config["image"]["width"].as<int>();
  }

  if (config["image"]["height"]) {
    height = config["image"]["height"].as<int>();
  }

  if (config["image"]["fps"]) {
    fps = config["image"]["fps"].as<float>();
  }

  if (config["image"]["publishTopic"]) {
    publishTopic = config["image"]["publishTopic"].as<std::string>();
  }

  avc::ImageCapture icap(nh, 0, width, height, fps, publishTopic);

  ros::Rate loop_rate(fps);

  while (nh.ok()) {
    icap.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
