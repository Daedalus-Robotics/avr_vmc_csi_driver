#ifndef CSI_DRIVER_CSI_DRIVER_HPP
#define CSI_DRIVER_CSI_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

namespace csi_driver
{
    class CSIDriverNode : public rclcpp::Node
    {
    public:
        explicit CSIDriverNode(const rclcpp::NodeOptions &);

        void release();

    private:
        static auto generateParamDescriptor(std::string description);

        bool isRunning = true;

        int captureWidth;
        int captureHeight;
        int captureFramerate;

        std::string pipeline;
        cv::VideoCapture capture;

        rclcpp::QoS videoQos;
        std_msgs::msg::Header header;
        std::shared_ptr<sensor_msgs::msg::CameraInfo> cameraInfo;

        sensor_msgs::msg::Image::SharedPtr imageRawMsg;
        image_transport::CameraPublisher imageRawPublisher;

        rclcpp::TimerBase::SharedPtr timer;

        cv::Mat imageRaw;

        void initParameters();

        void grabFrame();

        void publishFrame();


        static std::string gstreamerPipeline(int width, int height, int framerate, int flipMethod);
    };
}

#endif //CSI_DRIVER_CSI_DRIVER_HPP
