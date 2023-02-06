#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


class CSIStreamer : public rclcpp::Node
{
public:
    CSIStreamer() : Node("csi_driver"), capture(), videoQos(1), cameraInfo()
    {
        initParameters();

        captureWidth = (int) get_parameter("capture.width").get_parameter_value().get<int>();
        captureHeight = (int) get_parameter("capture.height").get_parameter_value().get<int>();
        int captureFramerate = (int) get_parameter("capture.framerate").get_parameter_value().get<int>();
        int captureFlipMethod = (int) get_parameter("capture.flip_method").get_parameter_value().get<int>();
        std::string opticalFrame = get_parameter("optical_frame").get_parameter_value().get<std::string>();

        pipeline = gstreamerPipeline(captureWidth, captureHeight, captureFramerate, captureFlipMethod);

        header.frame_id = opticalFrame;
        cameraInfo->width = captureWidth;
        cameraInfo->height = captureHeight;

        imageRawPublisher = image_transport::create_camera_publisher(this, "~/image_raw",
                                                                     videoQos.get_rmw_qos_profile());
        imageColorPublisher = image_transport::create_camera_publisher(this, "~/image_color",
                                                                       videoQos.get_rmw_qos_profile());
        imagePublisher = image_transport::create_camera_publisher(this, "~/image",
                                                                  videoQos.get_rmw_qos_profile());

        // Ensure that the timer is running slightly faster than the capture
        int durationMs = (1000 / (captureFramerate + 2));
        auto timerRate = std::chrono::milliseconds(durationMs);
        timer = this->create_wall_timer(timerRate, [this] { grabFrame(); });

        RCLCPP_INFO(this->get_logger(), "Opening camera (%dx%d, %dfps)...",
                    captureWidth, captureHeight, captureFramerate);
        capture.open(pipeline, cv::CAP_GSTREAMER);
    }

    void release()
    {
        isRunning = false;
        capture.release();
    }

private:
    static auto generateParamDescriptor(std::string description)
    {
        auto paramDescriptor = rcl_interfaces::msg::ParameterDescriptor{};
        paramDescriptor.description = std::move(description);
        paramDescriptor.read_only = true;

        return paramDescriptor;
    }


    bool isRunning = true;

    int captureWidth;
    int captureHeight;

    std::string pipeline;
    cv::VideoCapture capture;

    rclcpp::QoS videoQos;
    std_msgs::msg::Header header;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cameraInfo;

    sensor_msgs::msg::Image::SharedPtr imageRawMsg;
    image_transport::CameraPublisher imageRawPublisher;

    sensor_msgs::msg::Image::SharedPtr imageColorMsg;
    image_transport::CameraPublisher imageColorPublisher;

    sensor_msgs::msg::Image::SharedPtr imageMsg;
    image_transport::CameraPublisher imagePublisher;


    rclcpp::TimerBase::SharedPtr timer;

    cv::Mat imageRaw;
    cv::Mat imageColor;
    cv::Mat image;

    void initParameters()
    {
        declare_parameter("capture.width", 0, generateParamDescriptor(
                "Width in pixels of the camera")); // Should be specified by the launch file
        declare_parameter("capture.height", 0, generateParamDescriptor(
                "Height in pixels of the camera")); // Should be specified by the launch file
        declare_parameter("capture.framerate", 0, generateParamDescriptor(
                "Framerate of the camera")); // Should be specified by the launch file
        declare_parameter("capture.flip_method", 0, generateParamDescriptor(
                "Flip method: "
                "https://gstreamer.freedesktop.org/documentation/videofilter/videoflip.html#GstVideoFlipMethod"
        )); // Should be specified by the launch file

        declare_parameter("optical_frame", "csi_camera_optical_frame", generateParamDescriptor(
                "The tf2 frame id of the camera optical frame")); // Should be specified by the launch file
    }

    void grabFrame()
    {
        if (isRunning && capture.isOpened())
        {
            if (capture.grab())
            {
                cv::Mat imageGrayRaw;

                capture.retrieve(imageRaw);
                cv::cvtColor(imageRaw, imageGrayRaw, cv::COLOR_BGR2GRAY);

                // ToDo: Remove this and actually undistort the images
                cv::resize(imageRaw, imageColor, imageRaw.size());
                cv::resize(imageGrayRaw, image, imageRaw.size());

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

        imageRawMsg = cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::BGR8, imageRaw
        ).toImageMsg();
        imageColorMsg = cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::BGR8, imageColor
        ).toImageMsg();
        imageMsg = cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::MONO8, image
        ).toImageMsg();

        imageRawPublisher.publish(imageRawMsg, cameraInfo);
        imageColorPublisher.publish(imageColorMsg, cameraInfo);
        imagePublisher.publish(imageMsg, cameraInfo);
    }


    static std::string gstreamerPipeline(int width, int height, int framerate, int flipMethod)
    {
        return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width)
               + ", height=(int)" + std::to_string(height)
               + ", framerate=(fraction)" + std::to_string(framerate)
               + "/1 ! nvvidconv flip-method=" + std::to_string(flipMethod)
               + " ! video/x-raw, width=(int)" + std::to_string(width)
               + ", height=(int)" + std::to_string(height)
               + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<CSIStreamer> node = std::make_shared<CSIStreamer>();
    rclcpp::spin(node);
    node->release();
    rclcpp::shutdown();
    return 0;
}
