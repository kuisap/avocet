#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <vector>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <imu_capture.h>

namespace {
void showIMU(const avc::IMUData &data) {
  ROS_INFO_STREAM("[Acc] X = " << data.accX << ", Y = " << data.accY
                               << ", Z = " << data.accZ);
  ROS_INFO_STREAM("[Gyr] X = " << data.gyroX << ", Y = " << data.gyroY
                               << ", Z = " << data.gyroZ);
}

} // namespace

int main(int argc, char **argv) {
  float fps = 100;
  std::string frame_id = "base_link";
  std::string publishTopic = "/imu";
  std::string config_str = "./src/avocet_agv/config/config.yaml";

  ros::init(argc, argv, "avocet_imu");
  ros::NodeHandle nh("~");
  nh.getParam("config_str", config_str);

  {
    YAML::Node config = YAML::LoadFile(config_str);
    if (config["imu"]["fps"]) {
      fps = config["imu"]["fps"].as<float>();
    }

    if (config["imu"]["publishTopic"]) {
      publishTopic = config["imu"]["publishTopic"].as<std::string>();
    }

    if (config["imu"]["frame_id"]) {
      frame_id = config["imu"]["frame_id"].as<std::string>();
    }
  }

  avc::IMUCapture imu;
  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>(publishTopic, 1);

  ros::Rate loop_rate(fps);
  ROS_INFO_STREAM("IMU fps = " << fps << ", publish topic = " << publishTopic);
  while (nh.ok()) {
    const auto data = std::move(imu.sample());
    auto msg = sensor_msgs::Imu();
    {
      auto header = std_msgs::Header();
      header.stamp = ros::Time::now();
      header.frame_id = frame_id;
      msg.header = std::move(header);
    }

    for (auto i = 0; i < 9; i++) {
      msg.orientation_covariance[i] = -1;
    }

    msg.angular_velocity.x = std::move(data.gyroX);
    msg.angular_velocity.y = std::move(data.gyroY);
    msg.angular_velocity.z = std::move(data.gyroZ);
    for (auto i = 0; i < 9; i++) {
      msg.angular_velocity_covariance[i] = 0;
    }

    msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] =
        msg.angular_velocity_covariance[8] = 1;

    msg.linear_acceleration.x = std::move(data.accX);
    msg.linear_acceleration.y = std::move(data.accY);
    msg.linear_acceleration.z = std::move(data.accZ);
    for (auto i = 0; i < 9; i++) {
      msg.linear_acceleration_covariance[i] = 0;
    }
    msg.linear_acceleration_covariance[0] =
        msg.linear_acceleration_covariance[4] =
            msg.linear_acceleration_covariance[8] = 1;

    pub.publish(std::move(msg));

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
