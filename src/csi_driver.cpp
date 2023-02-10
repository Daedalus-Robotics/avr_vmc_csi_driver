#include "csi_driver/csi_driver.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

namespace csi_driver
{
    CSIDriverNode::CSIDriverNode(const rclcpp::NodeOptions &options) :
            Node("csi_driver", options),
            capture(),
            videoQos(1)
    {
        initParameters();

        captureWidth = (int) get_parameter("capture.width").get_parameter_value().get<int>();
        captureHeight = (int) get_parameter("capture.height").get_parameter_value().get<int>();
        captureFramerate = (int) get_parameter("capture.framerate").get_parameter_value().get<int>();
        RCLCPP_INFO(this->get_logger(), "Hi: fps:%d", captureFramerate);
        int captureFlipMethod = (int) get_parameter("capture.flip_method").get_parameter_value().get<int>();
        std::string opticalFrame = get_parameter("optical_frame").get_parameter_value().get<std::string>();

        pipeline = gstreamerPipeline(captureWidth, captureHeight, captureFramerate, captureFlipMethod);

        cameraInfo = std::make_shared<sensor_msgs::msg::CameraInfo>();

        header.frame_id = opticalFrame;
        cameraInfo->width = captureWidth;
        cameraInfo->height = captureHeight;

        imageRawPublisher = image_transport::create_camera_publisher(this, "image_raw",
                                                                     videoQos.get_rmw_qos_profile());

        // Ensure that the timer is running slightly faster than the capture
        assert(captureFramerate > 0);
        int durationMs = (1000 / (captureFramerate + 1));
        auto timerRate = std::chrono::milliseconds(durationMs);
        timer = this->create_wall_timer(timerRate, [this] { grabFrame(); });
    }

    void CSIDriverNode::release()
    {
        isRunning = false;
        capture.release();
    }

    auto CSIDriverNode::generateParamDescriptor(std::string description)
    {
        auto paramDescriptor = rcl_interfaces::msg::ParameterDescriptor{};
        paramDescriptor.description = std::move(description);
        paramDescriptor.read_only = true;

        return paramDescriptor;
    }

    void CSIDriverNode::initParameters()
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

    void CSIDriverNode::grabFrame()
    {
        if (isRunning && capture.isOpened())
        {
            if (capture.grab())
            {
                capture.retrieve(imageRaw);
                publishFrame();
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "No frame available");
            }
        }
        else
        {
            RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Could not open camera! Retrying...");
            RCLCPP_INFO(this->get_logger(), "Opening camera (%dx%d, %dfps)...",
                        captureWidth, captureHeight, captureFramerate);
            capture.open(pipeline, cv::CAP_GSTREAMER);
        }
    }

    void CSIDriverNode::publishFrame()
    {
        header.stamp = now();
        cameraInfo->header = header;

        imageRawMsg = cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::BGR8, imageRaw
        ).toImageMsg();

        imageRawPublisher.publish(imageRawMsg, cameraInfo);
    }


    std::string CSIDriverNode::gstreamerPipeline(int width, int height, int framerate, int flipMethod)
    {
        return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width)
               + ", height=(int)" + std::to_string(height)
               + ", framerate=(fraction)" + std::to_string(framerate)
               + "/1 ! nvvidconv flip-method=" + std::to_string(flipMethod)
               + " ! video/x-raw, width=(int)" + std::to_string(width)
               + ", height=(int)" + std::to_string(height)
               + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const rclcpp::NodeOptions options;
    std::shared_ptr<csi_driver::CSIDriverNode> node = std::make_shared<csi_driver::CSIDriverNode>(options);

    rclcpp::spin(node);

    node->release();

    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(csi_driver::CSIDriverNode)
