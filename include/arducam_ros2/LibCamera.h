#ifndef LIBCAM_HEAD_
#define LIBCAM_HEAD_

#include <atomic>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <limits.h>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <sstream>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <mutex>
#include <thread>

#include <libcamera/controls.h>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>
#include <libcamera/libcamera.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>
#include <libcamera/formats.h>
#include <libcamera/transform.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace libcamera;

typedef struct {
    uint8_t *imageData;
    uint32_t size;
    uint64_t request;
} LibcameraOutData;

class LibCamera {
    public:
        LibCamera(){};
        ~LibCamera(){};
        
        int initCamera();
        void configureStill(int width, int height, PixelFormat format, int buffercount);
        int startCamera();
        int resetCamera(int width, int height, PixelFormat format, int buffercount, int rotation);
        bool readFrame(LibcameraOutData *frameData);
        void returnFrameBuffer(LibcameraOutData frameData);

        void set(ControlList controls);
        void stopCamera();
        void closeCamera();

        Stream *VideoStream(uint32_t *w, uint32_t *h, uint32_t *stride) const;
        char * getCameraId();

    private:
        int startCapture();
        int queueRequest(Request *request);
        void requestComplete(Request *request);
        void processRequest(Request *request);

        void StreamDimensions(Stream const *stream, uint32_t *w, uint32_t *h, uint32_t *stride) const;

        unsigned int cameraIndex_;
	    uint64_t last_;
        std::unique_ptr<CameraManager> cm;
        std::shared_ptr<Camera> camera_;
        bool camera_acquired_ = false;
        bool camera_started_ = false;
	    std::unique_ptr<CameraConfiguration> config_;
        std::unique_ptr<FrameBufferAllocator> allocator_;
        std::vector<std::unique_ptr<Request>> requests_;
        // std::map<std::string, Stream *> stream_;
        std::map<int, std::pair<void *, unsigned int>> mappedBuffers_;

        std::queue<Request *> requestQueue;

        ControlList controls_;
        std::mutex control_mutex_;
        std::mutex camera_stop_mutex_;
        std::mutex free_requests_mutex_;

        Stream *viewfinder_stream_ = nullptr;
        std::string cameraId;
};

class ImgPublisher : public rclcpp::Node
{

    public:
        ImgPublisher();
        void th_impl_();

    private:
        
        LibCamera cam;
        ControlList controls_;
        LibcameraOutData frameData;

        uint32_t width = 5120;
        uint32_t height = 800;
        uint32_t stride;

        std::shared_ptr<std::thread> img_th_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_1_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_2_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_3_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_4_;
        
        cv_bridge::CvImagePtr cv_ptr;

        std::shared_ptr<cv::Mat> img1;
        std::shared_ptr<cv::Mat> img2;
        std::shared_ptr<cv::Mat> img3;
        std::shared_ptr<cv::Mat> img4;

        cv::Rect roi;

        sensor_msgs::msg::Image::SharedPtr msg_img_1_;
        sensor_msgs::msg::Image::SharedPtr msg_img_2_;
        sensor_msgs::msg::Image::SharedPtr msg_img_3_;
        sensor_msgs::msg::Image::SharedPtr msg_img_4_;

        std::shared_ptr<cv::Mat> img_whole_;

};

#endif