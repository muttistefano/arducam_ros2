#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "arducam_ros2/LibCamera.h"



ImgPublisher::ImgPublisher() : Node("ImgPublisher") {

    int ret = cam.initCamera();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("ImgPublisher"), "CAMERA --> " << ret);
    cam.configureStill(this->width, this->height, formats::R8, 1);

    int64_t frame_time = 1000000 / 50;
    controls_.set(controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({ frame_time, frame_time }));
    cam.set(controls_);

    pub_img_1_ = this->create_publisher<sensor_msgs::msg::Image>("img1", 1);
    pub_img_2_ = this->create_publisher<sensor_msgs::msg::Image>("img2", 1);
    pub_img_3_ = this->create_publisher<sensor_msgs::msg::Image>("img3", 1);
    pub_img_4_ = this->create_publisher<sensor_msgs::msg::Image>("img4", 1);
      
    img1 = std::make_shared<cv::Mat>(cv::Size(1280,800), CV_8UC1);
    img2 = std::make_shared<cv::Mat>(cv::Size(1280,800), CV_8UC1);
    img3 = std::make_shared<cv::Mat>(cv::Size(1280,800), CV_8UC1);
    img4 = std::make_shared<cv::Mat>(cv::Size(1280,800), CV_8UC1);

    img_whole_ = std::make_shared<cv::Mat>(height, width, CV_8UC1);

    img_th_ = std::make_shared<std::thread>(&ImgPublisher::th_impl_, this);

}

void ImgPublisher::th_impl_()
{

  bool flag;
  cam.startCamera();
  cam.VideoStream(&width, &height, &stride);

  while (rclcpp::ok()) {
    flag = cam.readFrame(&frameData);
    if (!flag)
        continue;

    img_whole_->data = frameData.imageData;

    // img1->data       = ((*img_whole_)(cv::Range(0, 800),cv::Range(0   , 5120))).data;
    // img2->data       = ((*img_whole_)(cv::Range(0, 800),cv::Range(1280, 2560))).data;
    // img3->data       = ((*img_whole_)(cv::Range(0, 800),cv::Range(2560, 3840))).data;
    // img4->data       = ((*img_whole_)(cv::Range(0, 800),cv::Range(3840, 5120))).data;
    cv::Mat pd1       = ((*img_whole_)(cv::Range(0, 800),cv::Range(0   , 1280)));
    cv::Mat pd2       = ((*img_whole_)(cv::Range(0, 800),cv::Range(1280, 2560)));
    cv::Mat pd3       = ((*img_whole_)(cv::Range(0, 800),cv::Range(2560, 3840)));
    cv::Mat pd4       = ((*img_whole_)(cv::Range(0, 800),cv::Range(3840, 5120)));

    msg_img_1_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", pd1).toImageMsg();
    msg_img_2_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", pd2).toImageMsg();
    msg_img_3_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", pd3).toImageMsg();
    msg_img_4_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", pd4).toImageMsg();

    auto now = this->get_clock()->now();

    msg_img_1_->header.stamp = now;
    msg_img_2_->header.stamp = now;
    msg_img_3_->header.stamp = now;
    msg_img_4_->header.stamp = now;

    msg_img_1_->header.frame_id = "cam1";
    msg_img_2_->header.frame_id = "cam2";
    msg_img_3_->header.frame_id = "cam3";
    msg_img_4_->header.frame_id = "cam4";

    pub_img_1_->publish(*msg_img_1_);
    pub_img_2_->publish(*msg_img_2_);
    pub_img_3_->publish(*msg_img_3_);
    pub_img_4_->publish(*msg_img_4_);

      cam.returnFrameBuffer(frameData);
  }

  RCLCPP_INFO(rclcpp::get_logger("ImgPublisher"), "STOPPING CAMERA");
  cam.stopCamera();
  cam.closeCamera();
    
}

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImgPublisher>());
  rclcpp::shutdown();

    
  return 0;
}
