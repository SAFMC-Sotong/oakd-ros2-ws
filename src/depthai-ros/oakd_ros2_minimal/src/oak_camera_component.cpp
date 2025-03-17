#include "../include/oak_camera_component.hpp"

#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace uosm
{
    namespace depthai
    {
        OakCamera::OakCamera(const rclcpp::NodeOptions &options)
            : Node("oak_node", options)
        {
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), "      OAK Camera Component ");
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
            RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
            RCLCPP_INFO(get_logger(), "********************************");

            std::chrono::milliseconds init_msec(static_cast<int>(100.0));
            mInitTimer = create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(init_msec),
                std::bind(&OakCamera::init, this));
        }

        void OakCamera::init()
        {
            RCLCPP_INFO(get_logger(), "Init params");

            mInitTimer->cancel();
            declare_parameter("monoResolution", "480p");
            declare_parameter("colorResolution", "1080p");
            declare_parameter("previewWidth", 300);
            declare_parameter("previewHeight", 300);
            declare_parameter("colorFPS", 15);
            declare_parameter("lrFPS", 30);
            declare_parameter("useRaw", false);
            declare_parameter("syncThreshold", 10);

            monoResolution = get_parameter("monoResolution").as_string();
            colorResolution = get_parameter("colorResolution").as_string();
            previewWidth = get_parameter("previewWidth").as_int();
            previewHeight = get_parameter("previewHeight").as_int();
            colorFPS = get_parameter("colorFPS").as_int();
            lrFPS = get_parameter("lrFPS").as_int();
            useRaw = get_parameter("useRaw").as_bool();
            syncThreshold = get_parameter("syncThreshold").as_int();

            declare_parameter("convInterleaved", false);
            declare_parameter("convGetBaseDeviceTimestamp", false);
            declare_parameter("convUpdateROSBaseTimeOnToRosMsg", true);
            declare_parameter("convReverseSocketOrder", true);
            convInterleaved = get_parameter("convInterleaved").as_bool();
            convGetBaseDeviceTimestamp = get_parameter("convGetBaseDeviceTimestamp").as_bool();
            convUpdateROSBaseTimeOnToRosMsg = get_parameter("convUpdateROSBaseTimeOnToRosMsg").as_bool();
            convReverseSocketOrder = get_parameter("convReverseSocketOrder").as_bool();

            declare_parameter("ffmpegBitrate", 5000);
            declare_parameter("ffmpegEncoder", "libx264");
            ffmpegBitrate = get_parameter("ffmpegBitrate").as_int();
            ffmpegEncoder = get_parameter("ffmpegEncoder").as_string();

            initPipeline();

            initPubSub();

            // start thread
            mPipelineThread = std::thread(&OakCamera::thread_OakPipeline, this);
        }

        OakCamera::~OakCamera()
        {
            RCLCPP_INFO(get_logger(), "Destroying node");

            RCLCPP_INFO(get_logger(), "Waiting for pipeline thread...");
            try
            {
                if (mPipelineThread.joinable())
                {
                    mPipelineThread.join();
                }
            }
            catch (std::system_error &e)
            {
                RCLCPP_INFO(get_logger(), "Pipeline thread joining exception");
            }
            RCLCPP_INFO(get_logger(), "... Pipeline thread stopped");
        }

        void OakCamera::initPipeline()
        {
            RCLCPP_INFO(get_logger(), "Initializing dai pipeline");
            pipeline = std::make_shared<dai::Pipeline>();
            // MonoCamera
            auto sync = pipeline->create<dai::node::Sync>();
            auto xoutGroup = pipeline->create<dai::node::XLinkOut>();
            auto monoLeft = pipeline->create<dai::node::MonoCamera>();
            auto monoRight = pipeline->create<dai::node::MonoCamera>();

            xoutGroup->setStreamName("xout");
            xoutGroup->input.setBlocking(false);
            sync->setSyncThreshold(std::chrono::milliseconds(syncThreshold));
            sync->setSyncAttempts(-1); // Infinite attempts

            dai::node::MonoCamera::Properties::SensorResolution monoResolutionProperties;
            if (monoResolution == "400p")
            {
                monoResolutionProperties = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
                monoWidth = 640;
                monoHeight = 400;
            }
            else if (monoResolution == "480p")
            {
                monoResolutionProperties = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
                monoWidth = 640;
                monoHeight = 480;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", monoResolution.c_str());
                throw std::runtime_error("Invalid mono camera resolution.");
            }
            monoLeft->setResolution(monoResolutionProperties);
            monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
            monoLeft->setFps(lrFPS);
            monoRight->setResolution(monoResolutionProperties);
            monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
            monoRight->setFps(lrFPS);

            auto manipLeft = pipeline->create<dai::node::ImageManip>();
            auto manipRight = pipeline->create<dai::node::ImageManip>();
            dai::RotatedRect rrLeft = {{monoLeft->getResolutionWidth() / 2.0f, monoLeft->getResolutionHeight() / 2.0f},
                                       {monoLeft->getResolutionWidth() * 1.0f, monoLeft->getResolutionHeight() * 1.0f},
                                       180};
            dai::RotatedRect rrRight = {{monoRight->getResolutionWidth() / 2.0f, monoRight->getResolutionHeight() / 2.0f},
                                        {monoRight->getResolutionWidth() * 1.0f, monoRight->getResolutionHeight() * 1.0f},
                                        180};
            manipLeft->initialConfig.setCropRotatedRect(rrLeft, false);
            monoLeft->out.link(manipLeft->inputImage);
            manipRight->initialConfig.setCropRotatedRect(rrRight, false);
            monoRight->out.link(manipRight->inputImage);

            if (useRaw)
            {
                manipLeft->out.link(sync->inputs["left"]);
                manipRight->out.link(sync->inputs["right"]);
            }
            else
            {
                auto stereo = pipeline->create<dai::node::StereoDepth>();
                stereo->initialConfig.setConfidenceThreshold(200);
                stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
                stereo->initialConfig.setLeftRightCheckThreshold(5);
                stereo->setLeftRightCheck(true);
                stereo->setExtendedDisparity(false);
                stereo->setSubpixel(true);
                manipLeft->out.link(stereo->left);
                manipRight->out.link(stereo->right);

                stereo->rectifiedLeft.link(sync->inputs["left"]);
                stereo->rectifiedRight.link(sync->inputs["right"]);
            }

            sync->out.link(xoutGroup->input);

            // ColorCamera
            auto colorCam = pipeline->create<dai::node::ColorCamera>();
            if (colorResolution == "1080p")
            {
                colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
                colorCam->setVideoSize(1920, 1080);
            }
            else if (colorResolution == "4K")
            {
                colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
                colorCam->setVideoSize(3840, 2160);
            }
            colorCam->setPreviewSize(previewWidth, previewHeight);
            colorCam->setVideoSize(previewWidth, previewHeight);
            colorCam->setInterleaved(false);
            colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
            colorCam->setFps(colorFPS);
            colorCam->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);

            auto videoEnc = pipeline->create<dai::node::VideoEncoder>();
            videoEnc->setDefaultProfilePreset(float(colorFPS), dai::VideoEncoderProperties::Profile::MJPEG);
            // videoEnc->setDefaultProfilePreset(colorFPS, dai::VideoEncoderProperties::Profile::H265_MAIN);
            // videoEnc->setBitrate(ffmpegBitrate);
            // videoEnc->setRateControlMode(dai::VideoEncoderProperties::RateControlMode::CBR);

            auto xlinkPreviewOut = pipeline->create<dai::node::XLinkOut>();
            xlinkPreviewOut->setStreamName("preview");
            // colorCam->preview.link(xlinkPreviewOut->input);
            colorCam->video.link(videoEnc->input);
            videoEnc->bitstream.link(xlinkPreviewOut->input);
        }

        void OakCamera::initPubSub()
        {
            RCLCPP_INFO(get_logger(), "Initializing publishers");
            rclcpp::PublisherOptions pubOptions;
            pubOptions.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

            // auto ffmpegQos = rclcpp::QoS(10);
            // ffmpegQos.reliability(rclcpp::ReliabilityPolicy::Reliable);
            // ffmpegQos.durability(rclcpp::DurabilityPolicy::Volatile);
            // ffmpegQos.history(rclcpp::HistoryPolicy::KeepLast);
            // rgbFFmpegPub = create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(RGB_TOPIC, ffmpegQos, pubOptions);
            // rgbImgPub = create_publisher<sensor_msgs::msg::Image>(RGB_TOPIC, rclcpp::QoS(10), pubOptions);
            rgbComprPub = create_publisher<sensor_msgs::msg::CompressedImage>(RGB_TOPIC, rclcpp::QoS(10), pubOptions);

            if (useRaw)
            {
                leftImgPub = create_publisher<sensor_msgs::msg::Image>(LEFT_TOPIC_RAW, rclcpp::QoS(10), pubOptions);
                rightImgPub = create_publisher<sensor_msgs::msg::Image>(RIGHT_TOPIC_RAW, rclcpp::QoS(10), pubOptions);
            }
            else
            {
                leftImgPub = create_publisher<sensor_msgs::msg::Image>(LEFT_TOPIC_RECT, rclcpp::QoS(10), pubOptions);
                rightImgPub = create_publisher<sensor_msgs::msg::Image>(RIGHT_TOPIC_RECT, rclcpp::QoS(10), pubOptions);
            }

            rgbInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>(RGB_INFO_TOPIC, rclcpp::QoS(10), pubOptions);
            leftInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>(LEFT_INFO_TOPIC, rclcpp::QoS(10), pubOptions);
            rightInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>(RIGHT_INFO_TOPIC, rclcpp::QoS(10), pubOptions);
        }

        void OakCamera::thread_OakPipeline()
        {
            RCLCPP_INFO(get_logger(), "Pipeline thread started");
            device = std::make_shared<dai::Device>(*pipeline, dai::UsbSpeed::SUPER_PLUS);
            auto calibrationHandler = device->readCalibration();

            auto leftConverter = std::make_shared<dai::ros::ImageConverter>("oak_left_camera_optical_frame", convInterleaved, convGetBaseDeviceTimestamp);
            auto rightConverter = std::make_shared<dai::ros::ImageConverter>("oak_right_camera_optical_frame", convInterleaved, convGetBaseDeviceTimestamp);
            auto rgbConverter = std::make_shared<dai::ros::ImageConverter>("oak_rgb_camera_optical_frame", convInterleaved, convGetBaseDeviceTimestamp);
            if (convUpdateROSBaseTimeOnToRosMsg)
            {
                leftConverter->setUpdateRosBaseTimeOnToRosMsg(convUpdateROSBaseTimeOnToRosMsg);
                rightConverter->setUpdateRosBaseTimeOnToRosMsg(convUpdateROSBaseTimeOnToRosMsg);
                rgbConverter->setUpdateRosBaseTimeOnToRosMsg(convUpdateROSBaseTimeOnToRosMsg);
            }

            if (convReverseSocketOrder)
            {
                leftConverter->reverseStereoSocketOrder();
                rightConverter->reverseStereoSocketOrder();
                rgbConverter->reverseStereoSocketOrder();
            }

            // rgbConverter->setFFMPEGEncoding(ffmpegEncoder);

            auto leftCameraInfo = leftConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
            auto rightCameraInfo = rightConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
            auto previewCameraInfo = rgbConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, previewWidth, previewHeight);

            groupQueue = device->getOutputQueue("xout", 8, false);
            rgbQueue = device->getOutputQueue("preview", 30, false);

            while (rclcpp::ok())
            {
                try
                {
                    // Process stereo messages
                    auto msgGrp = groupQueue->tryGet<dai::MessageGroup>();
                    if (msgGrp)
                    {
                        auto leftImgData = msgGrp->get<dai::ImgFrame>("left");
                        auto rightImgData = msgGrp->get<dai::ImgFrame>("right");

                        if (!leftImgData || !rightImgData)
                        {
                            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received incomplete stereo message group");
                            continue;
                        }

                        auto leftRawMsg = leftConverter->toRosMsgRawPtr(leftImgData, leftCameraInfo);
                        // leftRawMsg.frame_id = "oak_left_camera_optical_frame";
                        leftCameraInfo.header = leftRawMsg.header;

                        auto rightRawMsg = rightConverter->toRosMsgRawPtr(rightImgData, rightCameraInfo);
                        // rightRawMsg.header.frame_id = "oak_right_camera_optical_frame";
                        rightCameraInfo.header = rightRawMsg.header;

                        // Publish all data
                        leftImgPub->publish(leftRawMsg);
                        leftInfoPub->publish(leftCameraInfo);
                        rightImgPub->publish(rightRawMsg);
                        rightInfoPub->publish(rightCameraInfo);
                    }

                    auto rgbData = rgbQueue->tryGet<dai::ImgFrame>();
                    if(rgbData) {
                        auto rawMsg = rgbConverter->toRosCompressedMsg(rgbData);
                        previewCameraInfo.header = rawMsg.header;
                        rgbComprPub->publish(rawMsg);
                        rgbInfoPub->publish(previewCameraInfo);
                    }

                    // Short sleep to prevent CPU hogging
                    rclcpp::sleep_for(std::chrono::milliseconds(5));
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(get_logger(), "Standard exception in pipeline thread: %s", e.what());
                    continue;
                }
                catch (...)
                {
                    rcutils_reset_error();
                    RCLCPP_ERROR(get_logger(), "Unknown exception in pipeline thread");
                    continue;
                }
            }
            RCLCPP_INFO(get_logger(), "Pipeline thread finished");
        }

    } // namespace depthai
} // namespace uosm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::depthai::OakCamera)