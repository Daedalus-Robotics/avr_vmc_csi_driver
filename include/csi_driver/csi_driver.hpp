#ifndef CSI_DRIVER_CSI_DRIVER_HPP
#define CSI_DRIVER_CSI_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>

namespace csi_driver
{
    class CSIDriverNode : public rclcpp::Node
    {
    public:
        explicit CSIDriverNode(const rclcpp::NodeOptions &);

        void release();

    private:
        bool isRunning = true;

        int captureFramerate;
        int captureFlipMethod;

        std::chrono::duration<int64_t, std::milli> captureLoopPeriod{};
        std::chrono::steady_clock::time_point lastLoopTime;

        std::string pipeline;
        cv::VideoCapture capture;

        rclcpp::QoS videoQos;
        std_msgs::msg::Header header;
        std::shared_ptr<camera_info_manager::CameraInfoManager> cameraInfoManager;

        sensor_msgs::msg::Image::SharedPtr imageRawMsg;
        image_transport::CameraPublisher imageRawPublisher;

        rclcpp::TimerBase::SharedPtr timer;

        cv::Mat imageRaw;

        void populateCameraInfo();

        void initParameters();

        void grabFrame();

        void publishFrame();


        static rcl_interfaces::msg::ParameterDescriptor_<std::allocator<void>> generateParamDescriptor(
                std::string description
        );

        static std::string gstreamerPipeline(unsigned int width, unsigned int height, int framerate, int flipMethod);
    };
}

#endif //CSI_DRIVER_CSI_DRIVER_HPP
