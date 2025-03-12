#ifndef DM_VIO_ROS2__DM_VIO_ROS2_H_
#define DM_VIO_ROS2__DM_VIO_ROS2_H_

#include <mutex>
// DMVIO
#include "util/MainSettings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "live/FrameContainer.h"
#include "live/IMUInterpolator.h"
#include "live/FrameSkippingStrategy.h"
#include "IOWrapper/Output3DWrapper.h"
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_info_msgs/msg/image_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "dm_vio_msgs/msg/dmvio_pose.hpp"
#include "dm_vio_msgs/msg/dmvio_state.hpp"

namespace dmvio
{

    // We publish 3 topics by default:
    // dmvio/frame_tracked: DMVIOPoseMsg
    // dmvio/unscaled_pose: PoseStamped
    // dmvio/metric_poses: PoseStamped
    // For more details on these see the README.md file.
    class ROS2Wrapper : public dso::IOWrap::Output3DWrapper, public rclcpp::Node
    {
    public:
        ROS2Wrapper(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~ROS2Wrapper();

        /*
         * Usage:
         * Called once after each keyframe is optimized and passes the new transformation from DSO frame (worldToCam in
         * DSO scale) to metric frame (imuToWorld in metric scale).
         * Use transformPose of the passed object to transform poses between the frames.
         * Note that the object should not be used any more after the method returns.
         * The caller can create copy however , preferable with the following constructor (as otherwise the shared_ptrs will be kept).
         * TransformDSOToIMU(TransformDSOToIMU& other, std::shared_ptr<bool> optScale, std::shared_ptr<bool> optGravity, std::shared_ptr<bool> optT_cam_imu);
         */
        virtual void publishTransformDSOToIMU(const dmvio::TransformDSOToIMU &transformDSOToIMU) override;

        /*
         * Usage:
         * Called every time the status of the system changes.
         */
        virtual void publishSystemStatus(dmvio::SystemStatus systemStatus) override;

        /* Usage:
         * Called once for each tracked frame, with the real-time, low-delay frame pose.
         *
         * Calling:
         * Always called, no overhead if not used.
         */
        virtual void publishCamPose(dso::FrameShell *frame, dso::CalibHessian *HCalib) override;

        // In case you want to additionally publish pointclouds or keyframe poses you need to override Output3DWrapper::publishKeyframes

    private:
        void run();

        void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg_img, const image_info_msgs::msg::ImageInfo::ConstSharedPtr &msg_info);
        void callbackIMU(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

        // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionImage;
        message_filters::Subscriber<sensor_msgs::msg::Image> subscriptionImage;
        message_filters::Subscriber<image_info_msgs::msg::ImageInfo> subscriptionImageInfo;
        message_filters::TimeSynchronizer<sensor_msgs::msg::Image, image_info_msgs::msg::ImageInfo> syncImage;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriptionIMU;

        rclcpp::Publisher<dm_vio_msgs::msg::DMVIOState>::SharedPtr systemStatePublisher;
        rclcpp::Publisher<dm_vio_msgs::msg::DMVIOPose>::SharedPtr dmvioPosePublisher;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr unscaledPosePublisher, metricPosePublisher;

        std::string frame_world;

        // Protects transformDSOToIMU.
        std::mutex mutex;

        std::thread worker;

        std::unique_ptr<dmvio::TransformDSOToIMU> transformDSOToIMU;
        bool scaleAvailable = false; // True iff transformDSOToIMU contains a valid scale.
        std::atomic<dmvio::SystemStatus> lastSystemStatus;

        dmvio::FrameContainer frameContainer;
        dmvio::IMUInterpolator imuInt;
        std::unique_ptr<dso::Undistort> undistorter;
        std::unique_ptr<dso::FullSystem> fullSystem;
        std::unique_ptr<dmvio::FrameSkippingStrategy> frameSkipping;

        dmvio::MainSettings mainSettings;
        dmvio::IMUCalibration imuCalibration;
        dmvio::IMUSettings imuSettings;
        dmvio::FrameSkippingSettings frameSkippingSettings;

        bool stopSystem = false;
        size_t start = 2;
        //TODO read offset from basalt calibration
        double camTimeOffset = 0.0;
    };

}
#endif // DM_VIO_ROS2__DM_VIO_ROS2_H_
