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
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace dso
{
    namespace IOWrap
    {
        class KeyFrameDisplay;
    }
}

namespace dmvio
{
    using dso::IOWrap::KeyFrameDisplay;

    struct GraphConnection
    {
      KeyFrameDisplay *from;
      KeyFrameDisplay *to;
      int fwdMarg, bwdMarg, fwdAct, bwdAct;
    };

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

        virtual void pushLiveFrame(dso::FrameHessian *image) override;

        virtual bool needPushDepthImage() override { return true; };
        virtual void pushDepthImage(dso::MinimalImageB3 *image, dso::FrameHessian *KF) override;
        virtual void pushDepthImageFloat(dso::MinimalImageF *image, dso::FrameHessian *KF) override;

        virtual void publishKeyframes(std::vector<dso::FrameHessian *> &frames, bool final, dso::CalibHessian *HCalib) override;
        virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override;

    private:
        void reset_system();

        void run();

        void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg_img);
        void callbackImageExposure(const sensor_msgs::msg::Image::ConstSharedPtr &msg_img, const image_info_msgs::msg::ImageInfo::ConstSharedPtr &msg_info);
        void callbackIMU(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
        void callbackResetOrigin(const std_msgs::msg::Header::ConstSharedPtr &msg);
        void callbackResetOdometry(const std_msgs::msg::Header::ConstSharedPtr &msg);

        message_filters::Subscriber<sensor_msgs::msg::Image> sub_image;
        message_filters::Subscriber<image_info_msgs::msg::ImageInfo> sub_image_info;
        message_filters::TimeSynchronizer<sensor_msgs::msg::Image, image_info_msgs::msg::ImageInfo> sync_image;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

        rclcpp::Publisher<dm_vio_msgs::msg::DMVIOState>::SharedPtr pub_system_state;
        rclcpp::Publisher<dm_vio_msgs::msg::DMVIOPose>::SharedPtr pub_pose_dmvio;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_dso;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_live, pub_depth_image, pub_depth_float;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_keyframes, pub_pointcloud, pub_constraints, pub_trajectory;

        rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_reset_origin;
        rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_reset_origin;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tfbc_origin_odom;

        rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_reset_odometry;

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tfsbc_imu_camera;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tfbc_odom_base;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener;

        std::string frame_origin, frame_odom, frame_base, frame_imu, frame_camera;
        bool publish_tf, update_origin, reset_origin = false;
        double covariance_linear, covariance_angular;

        visualization_msgs::msg::Marker trajectory;

        // Protects transformDSOToIMU.
        std::mutex mutex;

        std::thread worker;

        std::unique_ptr<dmvio::TransformDSOToIMU> transformDSOToIMU;
        bool scaleAvailable = false; // True if transformDSOToIMU contains a valid scale.
        std::atomic<dmvio::SystemStatus> lastSystemStatus;
        double lastTimestamp = 0.0;
        Sophus::SE3d lastCamToWorld;

        dmvio::FrameContainer frameContainer;
        dmvio::IMUInterpolator imuInt;
        std::unique_ptr<dso::Undistort> undistorter;
        std::unique_ptr<dso::FullSystem> fullSystem;
        std::unique_ptr<dmvio::FrameSkippingStrategy> frameSkipping;

        dso::Settings dsoSettings;
        dmvio::MainSettings mainSettings;
        dmvio::IMUCalibration imuCalibration;
        dmvio::IMUSettings imuSettings;
        dmvio::FrameSkippingSettings frameSkippingSettings;

        bool stopSystem = false;
        size_t start = 2;
        double camTimeOffset = 0.0;

        // 3D model rendering
        std::unique_ptr<KeyFrameDisplay> currentCam;
        std::vector<KeyFrameDisplay *> keyframes;
        std::map<int, KeyFrameDisplay *> keyframesByKFID;
        std::vector<GraphConnection, Eigen::aligned_allocator<GraphConnection>> connections;
    };

}
#endif // DM_VIO_ROS2__DM_VIO_ROS2_H_
