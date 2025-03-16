#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

class VinsEstimatorNode : public rclcpp::Node
{
public:
    VinsEstimatorNode() : Node("vins_estimator")
    {
        // Declare parameters
        this->declare_parameter("config_file", "");
        this->declare_parameter("stereo_sync_tolerance", 0.003);

        // Get parameters
        std::string config_file = this->get_parameter("config_file").as_string();
        stereo_sync_tolerance_ = this->get_parameter("stereo_sync_tolerance").as_double();

        if (config_file.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Config file parameter is empty!");
            throw std::runtime_error("Config file parameter is empty!");
        }

        RCLCPP_INFO(this->get_logger(), "Config file: %s", config_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Stereo sync tolerance: %f", stereo_sync_tolerance_);

        // Initialize estimator
        readParameters(config_file);
        estimator_.setParameter();

        // Create subscribers
        if (USE_IMU)
        {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                IMU_TOPIC,
                rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort(), // Ensure QoS matches publisher
                std::bind(&VinsEstimatorNode::imuCallback, this, std::placeholders::_1));
        }

        feature_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(2000)),
            std::bind(&VinsEstimatorNode::featureCallback, this, std::placeholders::_1));

        img0_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE0_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)),
            std::bind(&VinsEstimatorNode::img0Callback, this, std::placeholders::_1));

        if (STEREO)
        {
            img1_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                IMAGE1_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)),
                std::bind(&VinsEstimatorNode::img1Callback, this, std::placeholders::_1));
        }

        restart_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/vins_restart", rclcpp::QoS(rclcpp::KeepLast(100)),
            std::bind(&VinsEstimatorNode::restartCallback, this, std::placeholders::_1));

        imu_switch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/vins_imu_switch", rclcpp::QoS(rclcpp::KeepLast(100)),
            std::bind(&VinsEstimatorNode::imuSwitchCallback, this, std::placeholders::_1));

        cam_switch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/vins_cam_switch", rclcpp::QoS(rclcpp::KeepLast(100)),
            std::bind(&VinsEstimatorNode::camSwitchCallback, this, std::placeholders::_1));

        // Start sync thread
        sync_thread_ = std::thread(&VinsEstimatorNode::syncProcess, this);

        RCLCPP_WARN(this->get_logger(), "VINS Estimator initialized, waiting for image and imu...");
    }

    ~VinsEstimatorNode()
    {
        if (sync_thread_.joinable())
        {
            sync_thread_.join();
        }
    }

private:
    void img0Callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        std::lock_guard<std::mutex> lock(m_buf_);
        img0_buf_.push(img_msg);
    }

    void img1Callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        std::lock_guard<std::mutex> lock(m_buf_);
        img1_buf_.push(img_msg);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * (1e-9);
        double dx = imu_msg->linear_acceleration.x;
        double dy = imu_msg->linear_acceleration.y;
        double dz = imu_msg->linear_acceleration.z;
        double rx = imu_msg->angular_velocity.x;
        double ry = imu_msg->angular_velocity.y;
        double rz = imu_msg->angular_velocity.z;
        Vector3d acc(dx, dy, dz);
        Vector3d gyr(rx, ry, rz);

        estimator_.inputIMU(t, acc, gyr);
    }

    void featureCallback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Feature callback: %zu points", feature_msg->points.size());

        std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
        for (unsigned int i = 0; i < feature_msg->points.size(); i++)
        {
            int feature_id = feature_msg->channels[0].values[i];
            int camera_id = feature_msg->channels[1].values[i];
            double x = feature_msg->points[i].x;
            double y = feature_msg->points[i].y;
            double z = feature_msg->points[i].z;
            double p_u = feature_msg->channels[2].values[i];
            double p_v = feature_msg->channels[3].values[i];
            double velocity_x = feature_msg->channels[4].values[i];
            double velocity_y = feature_msg->channels[5].values[i];

            if (feature_msg->channels.size() > 5)
            {
                double gx = feature_msg->channels[6].values[i];
                double gy = feature_msg->channels[7].values[i];
                double gz = feature_msg->channels[8].values[i];
                pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            }

            assert(z == 1);
            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
        }

        double t = feature_msg->header.stamp.sec + feature_msg->header.stamp.nanosec * (1e-9);
        estimator_.inputFeature(t, featureFrame);
    }

    void restartCallback(const std_msgs::msg::Bool::SharedPtr restart_msg)
    {
        if (restart_msg->data == true)
        {
            RCLCPP_WARN(this->get_logger(), "Restarting the estimator!");
            estimator_.clearState();
            estimator_.setParameter();
        }
    }

    void imuSwitchCallback(const std_msgs::msg::Bool::SharedPtr switch_msg)
    {
        if (switch_msg->data == true)
        {
            RCLCPP_INFO(this->get_logger(), "Using IMU");
            estimator_.changeSensorType(1, STEREO);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Disabling IMU");
            estimator_.changeSensorType(0, STEREO);
        }
    }

    void camSwitchCallback(const std_msgs::msg::Bool::SharedPtr switch_msg)
    {
        if (switch_msg->data == true)
        {
            RCLCPP_INFO(this->get_logger(), "Using stereo");
            estimator_.changeSensorType(USE_IMU, 1);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Using mono camera (left)");
            estimator_.changeSensorType(USE_IMU, 0);
        }
    }

    cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstPtr &img_msg)
    {
        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1")
        {
            sensor_msgs::msg::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
        {
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        }

        return ptr->image.clone();
    }

    void syncProcess()
    {
        while (rclcpp::ok())
        {
            if (STEREO)
            {
                cv::Mat image0, image1;
                std_msgs::msg::Header header;
                double time = 0;

                {
                    std::lock_guard<std::mutex> lock(m_buf_);
                    if (!img0_buf_.empty() && !img1_buf_.empty())
                    {
                        double time0 = img0_buf_.front()->header.stamp.sec + img0_buf_.front()->header.stamp.nanosec * (1e-9);
                        double time1 = img1_buf_.front()->header.stamp.sec + img1_buf_.front()->header.stamp.nanosec * (1e-9);

                        // Use the stereo sync tolerance parameter
                        if (time0 < time1 - stereo_sync_tolerance_)
                        {
                            img0_buf_.pop();
                            RCLCPP_DEBUG(this->get_logger(), "Discarding img0 (time diff: %f)", time1 - time0);
                        }
                        else if (time0 > time1 + stereo_sync_tolerance_)
                        {
                            img1_buf_.pop();
                            RCLCPP_DEBUG(this->get_logger(), "Discarding img1 (time diff: %f)", time0 - time1);
                        }
                        else
                        {
                            time = img0_buf_.front()->header.stamp.sec + img0_buf_.front()->header.stamp.nanosec * (1e-9);
                            header = img0_buf_.front()->header;
                            image0 = getImageFromMsg(img0_buf_.front());
                            img0_buf_.pop();
                            image1 = getImageFromMsg(img1_buf_.front());
                            img1_buf_.pop();
                        }
                    }
                }

                if (!image0.empty())
                {
                    estimator_.inputImage(time, image0, image1);
                }
            }
            else
            {
                cv::Mat image;
                std_msgs::msg::Header header;
                double time = 0;

                {
                    std::lock_guard<std::mutex> lock(m_buf_);
                    if (!img0_buf_.empty())
                    {
                        time = img0_buf_.front()->header.stamp.sec + img0_buf_.front()->header.stamp.nanosec * (1e-9);
                        header = img0_buf_.front()->header;
                        image = getImageFromMsg(img0_buf_.front());
                        img0_buf_.pop();
                    }
                }

                if (!image.empty())
                {
                    estimator_.inputImage(time, image);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    // Member variables
    Estimator estimator_;
    std::queue<sensor_msgs::msg::Imu::ConstPtr> imu_buf_;
    std::queue<sensor_msgs::msg::PointCloud::ConstPtr> feature_buf_;
    std::queue<sensor_msgs::msg::Image::ConstPtr> img0_buf_;
    std::queue<sensor_msgs::msg::Image::ConstPtr> img1_buf_;
    std::mutex m_buf_;
    std::thread sync_thread_;
    double stereo_sync_tolerance_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr feature_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img0_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img1_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr restart_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imu_switch_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cam_switch_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VinsEstimatorNode>();
    registerPub(node);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}