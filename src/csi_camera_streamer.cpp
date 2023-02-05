#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


std::string gstreamerPipeline(int capture_width, int capture_height,
                              int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width)
           + ", height=(int)" + std::to_string(capture_height)
           + ", framerate=(fraction)" + std::to_string(framerate)
           + "/1 ! nvvidconv flip-method=" + std::to_string(flip_method)
           + " ! video/x-raw, width=(int)" + std::to_string(display_width)
           + ", height=(int)" + std::to_string(display_height)
           + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}


class CSIStreamer : public rclcpp::Node
{
public:
    CSIStreamer() : Node("csi_camera_streamer"), capture(), videoQos(1)
    {
        pipeline = gstreamerPipeline(1280, 720,
                                     15, 0);

        header.frame_id = "csi_camera_frame";
        cameraInfo->width = 1280;
        cameraInfo->height = 720;

        colorPublisher = image_transport::create_camera_publisher(this, "~/image_color",
                                                                  videoQos.get_rmw_qos_profile());
        grayPublisher = image_transport::create_camera_publisher(this, "~/image_gray",
                                                                 videoQos.get_rmw_qos_profile());

        // Ensure that the timer is running slightly faster than the capture
        int durationMs = (1000 / (15 + 2));
        auto timerRate = std::chrono::milliseconds(durationMs);
        timer = this->create_wall_timer(timerRate, [this] { grabFrame(); });

        RCLCPP_INFO(this->get_logger(), "Opening camera (%dx%d, %dfps)...", 1280, 720, 15);
        capture.open(pipeline, cv::CAP_GSTREAMER);
    }

    void release()
    {
        running = false;
        capture.release();
    }

private:
    bool running = true;

    std::string pipeline;
    cv::VideoCapture capture;

    rclcpp::QoS videoQos;
    std_msgs::msg::Header header;
    std::shared_ptr <sensor_msgs::msg::CameraInfo> cameraInfo = std::make_shared <sensor_msgs::msg::CameraInfo>();

    sensor_msgs::msg::Image::SharedPtr imageColorMsg;
    image_transport::CameraPublisher colorPublisher;

    sensor_msgs::msg::Image::SharedPtr imageGrayMsg;
    image_transport::CameraPublisher grayPublisher;

    rclcpp::TimerBase::SharedPtr timer;

    cv::Mat frame;
    cv::Mat gray;

    void grabFrame()
    {
        if (running && capture.isOpened())
        {
            if (capture.grab())
            {
                capture.retrieve(frame);
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

                cv::resize(frame, frame, cv::Size(853, 480));
                cv::resize(gray, gray, cv::Size(853, 480));

                publishFrame();
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "No frame available");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Could not open camera! Retrying...");
            capture.open(pipeline, cv::CAP_GSTREAMER);
        }
    }

    void publishFrame()
    {
        header.stamp = now();
        cameraInfo->header = header;

        imageColorMsg = cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::BGR8, frame
        ).toImageMsg();
        imageGrayMsg = cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::MONO8, gray
        ).toImageMsg();

        colorPublisher.publish(imageColorMsg, cameraInfo);
        grayPublisher.publish(imageGrayMsg, cameraInfo);
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr <CSIStreamer> node = std::make_shared <CSIStreamer>();
    rclcpp::spin(node);
    node->release();
    rclcpp::shutdown();
    return 0;
}
