#include <cv_bridge/cv_bridge.h>
#include <image_capture.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

namespace avc {

ImageCapture::ImageCapture(ros::NodeHandle nh, size_t videoId, size_t width,
                           size_t height = 300, float fps,
                           const std::string& publishTopic)
  : impl_(std::move(
      std::make_unique<ImageCaptureImpl>(videoId, width, height, fps))) {}

ImageCapture::~ImageCapture() {}

class ImageCapture::ImageCaptureImpl {
public:
  ImageCaptureImpl(ros::NodeHandle nh, std::size_t videoId, std::size_t width,
                   std::size_t height, float fps,
                   const std::string& publishTopic)
    : videoId_(videoId)
    , width_(width)
    , height_(height)
    , fps_(fps)
    , publishTopic_(publishTopic) {
    image_transport::ImageTransport it(nh);
    pub_ = it.advertise(publishTopic_, 1);
    open();
  }

  void open() {
    cap_ = std::make_unique<cv::VideoCapture>(videoId_);
    if (!cap_->isOpened()) {
      ROS_ERROR_STREAM("[AVC Image Node] Camera could not be opened.");
      return 1;
    }
    cap_->set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_->set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_->set(cv::CAP_PROP_FPS, fps_);
    ROS_INFO_STREAM("[AVC Image Node] width = "
                    << width_ << ", height = " << height_ << ", fps = " << fps_
                    << ", publishTopic = " << publishTopic_);
  }

  void publish() {
    cv::Mat frame;
    *cap_ >> frame;
    if (!frame.empty()) {
      auto header = std_msgs::Header();
      header.stamp = ros::Time::now();
      ROS_INFO_STREAM("[AVC Image Node] Time " << header.stamp);
      pub_.publish(
        std::move(cv_bridge::CvImage(header, "bgr8", frame).toImageMsg()));
    }
    ROS_INFO_STREAM("[AVC Image Node] Time "
                    << ros::Time::now() << ", Topic " << publicTopic_
                    << " is not published because the image is not captured.");
  }

  void close() { cap_->release(); }

  ~ImageCaptureImpl() { close(); }

private:
  std::size_t videoId_;
  std::size_t width_;
  std::size_t height_;
  float fps_;
  image_transport::Publisher pub_;
  std::string publishTopic_;
  std::unique_ptr<cv::VideoCapture> cap_;
};

} // namespace avc
