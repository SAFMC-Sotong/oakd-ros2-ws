#ifndef OAK_CAMERA_COMPONENT_HPP_
#define OAK_CAMERA_COMPONENT_HPP_

#include "depthai/depthai.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"

#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <memory>
#include <string>
#include <vector>

namespace uosm
{
    namespace depthai
    {
        class OakCamera : public rclcpp::Node
        {
        public:
            explicit OakCamera(const rclcpp::NodeOptions &options);

            virtual ~OakCamera();

        protected:
            void init();
            void initPipeline();
            void initPubSub();

            void thread_OakPipeline();

        private:
            const std::string RGB_TOPIC = "rgb/compressed";
            const std::string LEFT_TOPIC_RAW = "left/image_raw";
            const std::string RIGHT_TOPIC_RAW = "right/image_raw";
            const std::string LEFT_TOPIC_RECT = "left/image_rect";
            const std::string RIGHT_TOPIC_RECT = "right/image_rect";

            const std::string RGB_INFO_TOPIC = "rgb/camera_info";
            const std::string LEFT_INFO_TOPIC = "left/camera_info";
            const std::string RIGHT_INFO_TOPIC = "right/camera_info";

            std::string monoResolution;
            std::string colorResolution;
            int monoWidth, monoHeight;
            int previewWidth, previewHeight;
            int colorFPS, lrFPS;
            int syncThreshold;
            bool useRaw;
            bool convInterleaved, convGetBaseDeviceTimestamp, convUpdateROSBaseTimeOnToRosMsg, convReverseSocketOrder;

            std::shared_ptr<dai::Pipeline> pipeline;
            std::shared_ptr<dai::Device> device;
            std::shared_ptr<dai::DataOutputQueue> groupQueue;
            std::shared_ptr<dai::DataOutputQueue> rgbQueue;

            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbImgPub, leftImgPub, rightImgPub;
            rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbInfoPub, leftInfoPub, rightInfoPub;
            rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr rgbFFmpegPub;

            rclcpp::TimerBase::SharedPtr mInitTimer;

            std::thread mPipelineThread;
            
        };

    } // namespace depthai
} // namespace uosm

#endif // OAK_CAMERA_COMPONENT_HPP_