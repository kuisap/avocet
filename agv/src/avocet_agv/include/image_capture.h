#pragma once
#include <ros/ros.h>

#include <cstddef>
#include <memory>

namespace avc {
class ImageCapture {
public:
  ImageCapture(ros::NodeHandle nh, size_t videoId = 0, size_t width = 300,
               size_t height = 300, float fps = 30.0F,
               const std::string& publishTopic = "/camera/image");
  ~ImageCapture();

  void publish();

private:
  class ImageCaptureImpl;
  std::unique_ptr<ImageCaptureImpl> impl_;
};
} // namespace avc
