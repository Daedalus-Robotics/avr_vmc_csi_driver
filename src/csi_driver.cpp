#include "csi_driver/csi_driver.hpp"

#include <rclcpp_components/register_node_macro.hpp>


namespace csi_driver
{
    CSIDriverNode::CSIDriverNode(const rclcpp::NodeOptions &options) :
            Node("csi_driver", options),
            capture(),
            videoQos(1)
    {
        initParameters();
        populateCameraInfo();

        captureFramerate = (int) get_parameter("framerate").get_parameter_value().get<int>();
        captureFlipMethod = (int) get_parameter("flip_method").get_parameter_value().get<int>();

        pipeline = gstreamerPipeline(
                cameraInfoManager->getCameraInfo().width,
                cameraInfoManager->getCameraInfo().height,
                captureFramerate,
                captureFlipMethod
        );
        RCLCPP_DEBUG(this->get_logger(), "Gstreamer pipeline: %s", pipeline.c_str());

        imageRawPublisher = image_transport::create_camera_publisher(this, "image_raw",
                                                                     videoQos.get_rmw_qos_profile());

        assert(captureFramerate > 0);
        // The normal framerate interval (used to calculate the frame drop)
        captureLoopPeriod = std::chrono::milliseconds(1000 / (captureFramerate));
        // Ensure that the timer is running slightly faster than the capture framerate
        // This ensures that we don't drop any frames
        auto timerRate = std::chrono::milliseconds(1000 / (captureFramerate + 10));
        RCLCPP_DEBUG(this->get_logger(), "Timer period: %ld", timerRate.count());
        timer = this->create_wall_timer(timerRate, [this] { grabFrame(); });

        lastLoopTime = std::chrono::steady_clock::now();
    }

    void CSIDriverNode::release()
    {
        isRunning = false;
        capture.release();
    }

    void CSIDriverNode::populateCameraInfo()
    {
        std::string cameraInfoPath = get_parameter("info_file").get_parameter_value().get<std::string>();
        bool noCameraInfo = cameraInfoPath.empty();
        cameraInfoPath = noCameraInfo ? "file://${ROS_HOME}/csi.yaml" : ("file://" + cameraInfoPath);

        sensor_msgs::msg::CameraInfo cameraInfo;

        cameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
                this,
                get_namespace(),
                cameraInfoPath
        );

        cameraInfo = cameraInfoManager->getCameraInfo();

        if (noCameraInfo)
        {
            RCLCPP_WARN(get_logger(), "No camera info file provided! Falling back to width and hight parameters.");
            RCLCPP_WARN(get_logger(), "This is not recommended and may result in incorrect camera info.");
            cameraInfo.width = (int) get_parameter("width").get_parameter_value().get<int>();
            cameraInfo.height = (int) get_parameter("height").get_parameter_value().get<int>();
        }
        header.frame_id = get_parameter("optical_frame").get_parameter_value().get<std::string>();
        cameraInfo.header = header;

        cameraInfoManager->setCameraInfo(cameraInfo);
    }

    void CSIDriverNode::initParameters()
    {
        declare_parameter("info_file", "", generateParamDescriptor(
                "Width in pixels of the camera")); // Should be specified by the launch file

        declare_parameter("width", 1920, generateParamDescriptor(
                "Width in pixels of the camera. "
                "Not used if there is a camera info file provided"));
        declare_parameter("height", 1080, generateParamDescriptor(
                "Height in pixels of the camera. "
                "Not used if there is a camera info file provided"));
        declare_parameter("framerate", 30, generateParamDescriptor(
                "Framerate of the camera")); // Should be specified by the launch file
        declare_parameter("flip_method", 0, generateParamDescriptor(
                "Flip method: "
                "https://gstreamer.freedesktop.org/documentation/videofilter/videoflip.html#GstVideoFlipMethod"
        )); // Should be specified by the launch file

        declare_parameter("optical_frame", "csi_camera_optical_frame", generateParamDescriptor(
                "The tf2 frame id of the camera optical frame")); // Should be specified by the launch file
    }

    void CSIDriverNode::grabFrame()
    {
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();

        long frameCount = (currentTime - lastLoopTime) / captureLoopPeriod - 1;
        if (frameCount > 0)
        {
            RCLCPP_WARN(this->get_logger(), "Grabbed frame too late! Dropping %ld frames...", frameCount);
        }
        lastLoopTime = currentTime;

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
                        cameraInfoManager->getCameraInfo().width,
                        cameraInfoManager->getCameraInfo().height,
                        captureFramerate
            );

            capture.open(pipeline, cv::CAP_GSTREAMER);
            capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
        }
    }

    void CSIDriverNode::publishFrame()
    {
        header.stamp = now();
        sensor_msgs::msg::CameraInfo cameraInfo = cameraInfoManager->getCameraInfo();
        cameraInfo.header = header;

        imageRawMsg = cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::BGR8, imageRaw
        ).toImageMsg();

        imageRawPublisher.publish(
                imageRawMsg,
                std::make_shared<sensor_msgs::msg::CameraInfo>(cameraInfo)
        );

#ifdef SHOW_FRAME
        cv::imshow("CSI Camera", imageRaw);
        cv::waitKey(1);
#endif
    }


    rcl_interfaces::msg::ParameterDescriptor_<std::allocator<void>> CSIDriverNode::generateParamDescriptor(
            std::string description
    )
    {
        rcl_interfaces::msg::ParameterDescriptor_<std::allocator<void>> paramDescriptor =
                rcl_interfaces::msg::ParameterDescriptor{};
        paramDescriptor.description = std::move(description);
        paramDescriptor.read_only = true;

        return paramDescriptor;
    }

    std::string CSIDriverNode::gstreamerPipeline(unsigned int width, unsigned int height, int framerate, int flipMethod)
    {
        std::string source = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width)
                             + ", height=(int)" + std::to_string(height)
                             + ", framerate=(fraction)" + std::to_string(framerate)
                             + "/1 ! nvvidconv flip-method=" + std::to_string(flipMethod)
                             + " ! video/x-raw, width=(int)" + std::to_string(width)
                             + ", height=(int)" + std::to_string(height)
                             + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR";

        std::string sink = " ! appsink drop=true, sync=false";
        return source + sink;
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
