/**
 * ROS driver for DM-VIO written by Lukas von Stumberg (http://vision.in.tum.de/dm-vio).
 *
 * Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
 * for more information see <http://vision.in.tum.de/dm-vio>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DM-VIO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DM-VIO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DM-VIO. If not, see <http://www.gnu.org/licenses/>.
 */

#include "filesystem"
#include "dm_vio_ros2/dm_vio_ros2.hpp"
#include "keyframe_display.hpp"
#include "util/FrameShell.h"
#include "util/TimeMeasurement.h"
#include "GTSAMIntegration/PoseTransformationIMU.h"
#include "cv_bridge/cv_bridge.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "basalt/calibration/calibration.hpp"
#include "basalt/serialization/headers_serialization.h"

#ifndef NDEBUG
#define RCLCPP_NDEBUG(logger, fmt, ...) RCLCPP_DEBUG(logger, fmt, __VA_ARGS__)
#else
#define RCLCPP_NDEBUG(logger, fmt, ...)
#endif

namespace dmvio
{
    inline void setTransformFromSE3(const Sophus::SE3d &se3, geometry_msgs::msg::Transform &transform)
    {
        const auto translation = se3.translation();
        const auto rotation = se3.unit_quaternion();
        transform.translation.x = translation.x();
        transform.translation.y = translation.y();
        transform.translation.z = translation.z();
        transform.rotation.x = rotation.x();
        transform.rotation.y = rotation.y();
        transform.rotation.z = rotation.z();
        transform.rotation.w = rotation.w();
    }

    inline void setTransform2FromSE3(const Sophus::SE3d &se3, tf2::Transform &transform)
    {
        const auto translation = se3.translation();
        const auto rotation = se3.unit_quaternion();
        transform.setOrigin(tf2::Vector3(translation.x(), translation.y(), translation.z()));
        transform.setRotation(tf2::Quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()));
    }

    inline void setPoseFromSE3(const Sophus::SE3d &se3, geometry_msgs::msg::Pose &pose)
    {
        const auto translation = se3.translation();
        const auto rotation = se3.unit_quaternion();
        pose.position.x = translation.x();
        pose.position.y = translation.y();
        pose.position.z = translation.z();
        pose.orientation.x = rotation.x();
        pose.orientation.y = rotation.y();
        pose.orientation.z = rotation.z();
        pose.orientation.w = rotation.w();
    }

    inline void setCovarianceMatrixFromValues(double linear, double angular, std::array<double, 36> &covariance)
    {
        covariance[0] = linear;
        covariance[7] = linear;
        covariance[14] = linear;
        covariance[21] = angular;
        covariance[28] = angular;
        covariance[35] = angular;
    }
    inline void setCovarianceMatrixFromVectors(const Eigen::Vector3d &linear, const Eigen::Vector3d &angular, std::array<double, 36> &covariance)
    {
        covariance[0] = linear.x();
        covariance[7] = linear.y();
        covariance[14] = linear.z();
        covariance[21] = angular.x();
        covariance[28] = angular.y();
        covariance[35] = angular.z();
    }
    inline void setCovarianceMatrixFromVector(const Eigen::Vector<double, 6> &variance, std::array<double, 36> &covariance)
    {
        covariance[0] = variance[0];
        covariance[7] = variance[1];
        covariance[14] = variance[2];
        covariance[21] = variance[3];
        covariance[28] = variance[4];
        covariance[35] = variance[5];
    }

    inline void setCovarianceMatrixFromTwist(const geometry_msgs::msg::Twist &twist, double scale_linear, double scale_angular, std::array<double, 36> &covariance)
    {
        covariance[0] = std::abs(twist.linear.x) * scale_linear;
        covariance[7] = std::abs(twist.linear.y) * scale_linear;
        covariance[14] = std::abs(twist.linear.z) * scale_linear;
        covariance[21] = std::abs(twist.angular.x) * scale_angular;
        covariance[28] = std::abs(twist.angular.y) * scale_angular;
        covariance[35] = std::abs(twist.angular.z) * scale_angular;
    }

    inline rclcpp::Time stampFromDSO(double timestamp)
    {
        return rclcpp::Time(timestamp * 1e9);
    }

    void ROS2Wrapper::reset_system()
    {
        this->fullSystem = std::make_unique<dso::FullSystem>(this->mainSettings.playbackSpeed == 0, this->imuCalibration, this->imuSettings, &(this->dsoSettings));

        if (this->dsoSettings.photometricCalibration > 0 && this->undistorter->photometricUndist == nullptr)
        {
            RCLCPP_ERROR(get_logger(), "Photometric calibration not available! Need to use mode=1 or mode=2");
            throw std::invalid_argument("Photometric calibration not available!");
        }

        if (this->undistorter->photometricUndist != nullptr)
        {
            this->fullSystem->setGammaFunction(this->undistorter->photometricUndist->getG());
        }

        this->frameSkipping->reset();

        for (auto &keyframe : this->keyframes)
            delete keyframe;
        this->keyframes.clear();
        this->keyframesByKFID.clear();
        this->connections.clear();

        this->fullSystem->outputWrapper.push_back(this->frameSkipping.get());
        this->fullSystem->outputWrapper.push_back(this);
    }

    ROS2Wrapper::ROS2Wrapper(const rclcpp::NodeOptions &options)
        : Node("dm_vio", options), imuInt(this->frameContainer, nullptr), mainSettings(&(this->dsoSettings))
    {
        // declare params
        this->declare_parameter("calibration", "");
        this->declare_parameter("camera_index", 0);
        this->declare_parameter("projection", std::vector<double>());
        this->declare_parameter("resolution", std::vector<int>());

        this->declare_parameter("mode", 0);
        this->declare_parameter("preset", 0);
        this->declare_parameter("use_imu", true);
        this->declare_parameter("use_exposure", true);
        this->declare_parameter("quiet", true);
        this->declare_parameter("nolog", true);
        this->declare_parameter("results_path", std::filesystem::temp_directory_path() / "dm_vio_results");
        this->declare_parameter("max_skip_visual_init", 0);
        this->declare_parameter("max_skip_visual_only", 1);
        this->declare_parameter("max_skip_visual_inertial", 2);
        this->declare_parameter("max_skip_full_reset", -1);
        this->declare_parameter("skip_delay_visual_only", 20);
        this->declare_parameter("imu_noise_factor", 160.0);
        this->declare_parameter("imu_bias_factor", 500.0);
        this->declare_parameter("pgba_skip_first_kfs", 1);
        this->declare_parameter("init_normalized_error_threshold", 1.0);
        this->declare_parameter("max_time_between_init_frames", 1.0);
        this->declare_parameter("start_skip", 2);
        this->declare_parameter("candidate_points", 0);
        this->declare_parameter("active_points", 0);

        this->declare_parameter("frame_origin", "origin");
        this->declare_parameter("frame_odom", "odom");
        this->declare_parameter("frame_base", "base");
        this->declare_parameter("frame_imu", "imu");
        this->declare_parameter("frame_camera", "camera");
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("update_origin", false);

        this->declare_parameter("covariance_linear", 0.001);
        this->declare_parameter("covariance_angular", 0.001);
        this->declare_parameter("covariance_linear_scaling", 0.0);
        this->declare_parameter("covariance_angular_scaling", 0.0);

        // get params
        std::string calib_path = this->get_parameter("calibration").as_string();
        this->dsoSettings.multiCameraIndex = this->get_parameter("camera_index").as_int();
        std::vector<double> projection = this->get_parameter("projection").as_double_array();
        if (projection.size() == 1 && projection[0] < 0)
            projection.clear();
        if (projection.size() != 0 && projection.size() != 1 && projection.size() != 2 && projection.size() != 4)
            throw std::invalid_argument("Projection must be empty or a vector defined as [f_x [, fy [, cx, cy]]]!");
        std::vector<int64_t> resolution = this->get_parameter("resolution").as_integer_array();

        int mode = this->get_parameter("mode").as_int();
        int preset = this->get_parameter("preset").as_int();
        bool quiet = this->get_parameter("quiet").as_bool();
        bool nolog = this->get_parameter("nolog").as_bool();
        bool use_imu = this->get_parameter("use_imu").as_bool();
        bool use_exposure = this->get_parameter("use_exposure").as_bool();
        double imu_noise_factor = this->get_parameter("imu_noise_factor").as_double();
        double imu_bias_factor = this->get_parameter("imu_bias_factor").as_double();
        int candidate_points = this->get_parameter("candidate_points").as_int();
        int active_points = this->get_parameter("active_points").as_int();
        this->start = this->get_parameter("start_skip").as_int();
        this->imuSettings.resultsPrefix = std::filesystem::path(this->get_parameter("results_path").as_string()).string() + '/';
        this->imuSettings.initSettings.pgbaSettings.skipFirstKFs = this->get_parameter("pgba_skip_first_kfs").as_int();
        this->imuSettings.initSettings.coarseInitSettings.requestFullResetNormalizedErrorThreshold = this->get_parameter("init_normalized_error_threshold").as_double();
        this->imuSettings.maxTimeBetweenInitFrames = this->get_parameter("max_time_between_init_frames").as_double();
        this->frameSkippingSettings.maxSkipFramesVisualInit = this->get_parameter("max_skip_visual_init").as_int();
        this->frameSkippingSettings.maxSkipFramesVisualOnlyMode = this->get_parameter("max_skip_visual_only").as_int();
        this->frameSkippingSettings.maxSkipFramesVisualInertial = this->get_parameter("max_skip_visual_inertial").as_int();
        this->frameSkippingSettings.maxSkipFramesFullReset = this->get_parameter("max_skip_full_reset").as_int();
        this->frameSkippingSettings.skipFramesVisualOnlyDelay = this->get_parameter("skip_delay_visual_only").as_int();

        this->frame_origin = this->get_parameter("frame_origin").as_string();
        this->frame_odom = this->get_parameter("frame_odom").as_string();
        this->frame_base = this->get_parameter("frame_base").as_string();
        this->frame_imu = this->get_parameter("frame_imu").as_string();
        this->frame_camera = this->get_parameter("frame_camera").as_string();
        this->publish_tf = this->get_parameter("publish_tf").as_bool();
        this->update_origin = this->get_parameter("update_origin").as_bool();

        this->covariance_linear = this->get_parameter("covariance_linear").as_double();
        this->covariance_angular = this->get_parameter("covariance_angular").as_double();
        this->covariance_linear_scaling = this->get_parameter("covariance_linear_scaling").as_double();
        this->covariance_angular_scaling = this->get_parameter("covariance_angular_scaling").as_double();

        if (calib_path.empty())
            throw std::invalid_argument("Calibration path not set!");

        std::filesystem::create_directory(this->imuSettings.resultsPrefix);
        if (!std::filesystem::exists(this->imuSettings.resultsPrefix))
            throw std::invalid_argument("Results path not found!");

        std::ifstream calib_file(calib_path);
        if (!calib_file.good())
            throw std::invalid_argument("Calibration file not found!");

        // apply params
        basalt::Calibration<double> calib;
        cereal::JSONInputArchive archive(calib_file);
        archive(calib);
        this->camTimeOffset = calib.cam_time_offset_ns * 1e-9;

        switch (mode)
        {
        case 0:
            RCLCPP_INFO(get_logger(), "Mode: Photometric correction with calibration");
            break;
        case 1:
            RCLCPP_INFO(get_logger(), "Mode: Photometric correction without calibration");
            this->dsoSettings.photometricCalibration = 0;
            this->dsoSettings.affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            this->dsoSettings.affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            break;
        case 2:
            RCLCPP_INFO(get_logger(), "Mode: Perfect images (no photometric correction)");
            this->dsoSettings.photometricCalibration = 0;
            this->dsoSettings.affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            this->dsoSettings.affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            this->dsoSettings.minGradHistAdd = 3;
            break;
        case 3:
            RCLCPP_INFO(get_logger(), "Mode: Photometric correction with calibration, but no or incorrect exposure times");
            this->dsoSettings.affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            this->dsoSettings.affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            break;
        default:
            throw std::invalid_argument("Photometric correction mode unknown");
        }

        this->mainSettings.settingsDefault(preset);
        if (candidate_points > 0)
        {
            this->dsoSettings.desiredImmatureDensity = candidate_points;
            RCLCPP_INFO(get_logger(), "Set candidate points to %d", candidate_points);
        }
        if (active_points > 0)
        {
            this->dsoSettings.desiredPointDensity = active_points;
            RCLCPP_INFO(get_logger(), "Set active points to %d", active_points);
        }

        if (quiet)
        {
            this->dsoSettings.debugout_runquiet = true;
            RCLCPP_INFO(get_logger(), "Enabled: Quiet Mode");
        }
        if (nolog)
        {
            this->dsoSettings.logStuff = false;
            RCLCPP_INFO(get_logger(), "Disabled: Internal Logging");
        }
        if (use_imu)
        {
            RCLCPP_INFO(get_logger(), "Enabled: IMU integration");
            this->dsoSettings.useIMU = true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Disabled: IMU integration");
            this->dsoSettings.useIMU = false;
        }
        if (use_exposure)
        {
            RCLCPP_INFO(get_logger(), "Enabled: Exposure integration");
            this->dsoSettings.useExposure = true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Disabled: Exposure integration");
            this->dsoSettings.useExposure = false;
        }

        if (!resolution.empty())
        {
            this->dsoSettings.wTarget = resolution[0];
            this->dsoSettings.hTarget = resolution.size() == 1 ? resolution[0] : resolution[1];
        }
        this->undistorter.reset(dso::Undistort::makeFromBasaltCalibration(&dsoSettings, calib_path, projection));

        this->dsoSettings.calibG = dso::GlobalCalib(
            this->undistorter->getSize()[0],
            this->undistorter->getSize()[1],
            this->undistorter->getK().cast<float>(),
            this->dsoSettings.pyrLevelsUsed);

        this->imuCalibration.loadFromFile(calib_path, this->dsoSettings.multiCameraIndex, imu_noise_factor, imu_bias_factor);

        // setup ros
        const auto setup_time = get_clock()->now();

        auto sub_options_imu = rclcpp::SubscriptionOptions();
        sub_options_imu.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options_image = rclcpp::SubscriptionOptions();
        sub_options_image.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        this->sub_img = this->create_subscription<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS(), std::bind(&ROS2Wrapper::callbackImage, this, std::placeholders::_1), sub_options_image);
        this->sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(), std::bind(&ROS2Wrapper::callbackIMU, this, std::placeholders::_1), sub_options_imu);

        this->pub_system_state = this->create_publisher<dm_vio_msgs::msg::DMVIOState>("tracking_state", rclcpp::SensorDataQoS());
        this->pub_pose_dmvio = this->create_publisher<dm_vio_msgs::msg::DMVIOPose>("pose_dmvio", rclcpp::SensorDataQoS());
        this->pub_pose_dso = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_dso", rclcpp::SensorDataQoS());
        this->pub_odometry = this->create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::SensorDataQoS());

        this->pub_image_live = this->create_publisher<sensor_msgs::msg::Image>("image_live", rclcpp::SensorDataQoS());
        this->pub_depth_image = this->create_publisher<sensor_msgs::msg::Image>("image_depth", rclcpp::SensorDataQoS());
        this->pub_depth_float = this->create_publisher<sensor_msgs::msg::Image>("image_depth_float", rclcpp::SensorDataQoS());

        this->pub_keyframes = this->create_publisher<visualization_msgs::msg::Marker>("keyframes", rclcpp::SystemDefaultsQoS());
        this->pub_pointcloud = this->create_publisher<visualization_msgs::msg::Marker>("pointscloud", rclcpp::SystemDefaultsQoS());
        this->pub_constraints = this->create_publisher<visualization_msgs::msg::Marker>("constraints", rclcpp::SystemDefaultsQoS());
        this->pub_trajectory = this->create_publisher<visualization_msgs::msg::Marker>("trajectory", rclcpp::SystemDefaultsQoS());

        this->sub_reset_odometry = this->create_subscription<std_msgs::msg::Header>("reset_odometry", rclcpp::ServicesQoS(), std::bind(&ROS2Wrapper::callbackResetOdometry, this, std::placeholders::_1));

        if (this->update_origin)
            this->pub_reset_origin = this->create_publisher<std_msgs::msg::Header>("reset_origin", rclcpp::ServicesQoS());
        this->sub_reset_origin = this->create_subscription<std_msgs::msg::Header>("reset_origin", rclcpp::ServicesQoS(), std::bind(&ROS2Wrapper::callbackResetOrigin, this, std::placeholders::_1));
        this->tfbc_origin_odom = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        this->tfsbc_imu_camera = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        this->tfbc_odom_base = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf_buffer, this, false);

        trajectory.header.frame_id = this->frame_odom;
        trajectory.ns = "frame";
        trajectory.id = 0;
        trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trajectory.action = visualization_msgs::msg::Marker::MODIFY;
        trajectory.color.r = 1.0;
        trajectory.color.g = 0.0;
        trajectory.color.b = 0.0;
        trajectory.color.a = 1.0;
        trajectory.scale.x = 0.03;

        // publish static data
        geometry_msgs::msg::TransformStamped tf_origin_odom;
        tf_origin_odom.header.stamp = setup_time;
        tf_origin_odom.header.frame_id = this->frame_origin;
        tf_origin_odom.child_frame_id = this->frame_odom;
        tf2::toMsg(tf2::Transform::getIdentity(), tf_origin_odom.transform);
        tfbc_origin_odom->sendTransform(tf_origin_odom);

        geometry_msgs::msg::TransformStamped tf_imu_camera;
        tf_imu_camera.header.stamp = setup_time;
        tf_imu_camera.header.frame_id = this->frame_imu;
        tf_imu_camera.child_frame_id = this->frame_camera;
        setTransformFromSE3(calib.T_i_c[this->dsoSettings.multiCameraIndex], tf_imu_camera.transform);
        tfsbc_imu_camera->sendTransform(tf_imu_camera);

        rclcpp::PublisherOptions static_pub_options;
        static_pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        this->pub_camera_info = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::QoS(1).reliable().transient_local(), static_pub_options);
        auto camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
        camera_info->header.stamp = setup_time;
        camera_info->header.frame_id = this->frame_camera;
        camera_info->width = this->dsoSettings.calibG.wG[0];
        camera_info->height = this->dsoSettings.calibG.hG[0];
        camera_info->k[0] = this->dsoSettings.calibG.KG[0](0, 0);
        camera_info->k[2] = this->dsoSettings.calibG.KG[0](0, 2);
        camera_info->k[4] = this->dsoSettings.calibG.KG[0](1, 1);
        camera_info->k[5] = this->dsoSettings.calibG.KG[0](1, 2);
        camera_info->k[8] = 1.0;
        this->pub_camera_info->publish(std::move(camera_info));

        // setup vio
        this->currentCam = std::make_unique<KeyFrameDisplay>(&(this->dsoSettings), "frame", this->frame_odom);
        this->frameSkipping = std::make_unique<dmvio::FrameSkippingStrategy>(this->frameSkippingSettings);
        this->reset_system();
        this->worker = std::thread(std::bind(&ROS2Wrapper::run, this));
    }

    ROS2Wrapper::~ROS2Wrapper()
    {
        RCLCPP_INFO(get_logger(), "Shutting down");
        this->stopSystem = true;
        this->frameContainer.stop();
        this->worker.join();
    }

    void ROS2Wrapper::publishTransformDSOToIMU(const TransformDSOToIMU &transformDSOToIMUPassed)
    {
        std::unique_lock<std::mutex> lk(mutex);
        this->transformDSOToIMU = std::make_unique<dmvio::TransformDSOToIMU>(transformDSOToIMUPassed,
                                                                             std::make_shared<bool>(false),
                                                                             std::make_shared<bool>(false),
                                                                             std::make_shared<bool>(false));
        this->scaleAvailable = lastSystemStatus == SystemStatus::VISUAL_INERTIAL;
    }

    void ROS2Wrapper::publishSystemStatus(dmvio::SystemStatus systemStatus)
    {
        auto msg = std::make_unique<dm_vio_msgs::msg::DMVIOState>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = this->frame_camera;
        msg->state = static_cast<int>(systemStatus);

        if (systemStatus == dmvio::SystemStatus::VISUAL_INERTIAL)
        {
            RCLCPP_INFO(get_logger(), "System status: VISUAL_INERTIAL");
            if (this->update_origin)
                this->pub_reset_origin->publish(msg->header);
        }
        else if (systemStatus == dmvio::SystemStatus::VISUAL_ONLY)
        {
            RCLCPP_INFO(get_logger(), "System status: VISUAL_ONLY");
        }
        else if (systemStatus == dmvio::SystemStatus::VISUAL_INIT)
        {
            RCLCPP_INFO(get_logger(), "System status: VISUAL_INIT");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "System status: UNKNOWN");
        }
        this->pub_system_state->publish(std::move(msg));
        this->lastSystemStatus = systemStatus;
    }

    void ROS2Wrapper::publishCamPose(dso::FrameShell *frame, dso::CalibHessian *HCalib)
    {
        auto msg = std::make_unique<dm_vio_msgs::msg::DMVIOPose>();
        msg->header.stamp = stampFromDSO(frame->timestamp);
        msg->header.frame_id = this->frame_odom;

        const auto &camToWorld = frame->camToWorld;
        setPoseFromSE3(camToWorld, msg->pose);

        // Also publish unscaled pose on its own (e.g. for visualization in Rviz).
        auto unscaledMsg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        unscaledMsg->header = msg->header;
        unscaledMsg->pose = msg->pose;
        this->pub_pose_dso->publish(std::move(unscaledMsg));

        {
            std::unique_lock<std::mutex> lk(mutex);
            if (this->transformDSOToIMU && this->scaleAvailable)
            {
                msg->scale = this->transformDSOToIMU->getScale();

                // Transform to metric imu to world. Note that we need to use the inverse as transformDSOToIMU expects
                // worldToCam as an input!
                const auto imuToWorld = Sophus::SE3d(this->transformDSOToIMU->transformPose(camToWorld.inverse().matrix()));

                // Get transfrom from imu to base frame, to publish odometry to base frame, when available.
                bool tf2_imu_base_available;
                tf2::Transform tf2_imu_base;
                try
                {
                    const auto tf_imu_base = this->tf_buffer->lookupTransform(this->frame_base, this->frame_imu, tf2::TimePointZero);
                    tf2::fromMsg(tf_imu_base.transform, tf2_imu_base);
                    tf2_imu_base_available = true;
                }
                catch (const tf2::TransformException &e)
                {
                    RCLCPP_WARN_ONCE(get_logger(), "Transform exception: %s, Falling back to identity.", e.what());
                    tf2_imu_base.setIdentity();
                    tf2_imu_base_available = false;
                }
                tf2::Transform tf2_imu_odom;
                setTransform2FromSE3(imuToWorld.inverse(), tf2_imu_odom);
                tf2::Transform tf2_odom_base = (tf2_imu_base * tf2_imu_odom).inverse();

                if (this->initTimestamp == 0)
                    this->initTimestamp = frame->timestamp;
                if (this->lastTimestamp > 0)
                {
                    // Publish odometry
                    auto odomMsg = std::make_unique<nav_msgs::msg::Odometry>();
                    odomMsg->header = msg->header;
                    odomMsg->child_frame_id = this->frame_imu;

                    tf2::toMsg(tf2_odom_base, odomMsg->pose.pose);

                    // Compute velocity
                    const auto lastImuToWorld = Sophus::SE3d(this->transformDSOToIMU->transformPose(this->lastCamToWorld.inverse().matrix()));

                    const auto diffTransform = lastImuToWorld.inverse() * imuToWorld;
                    const auto diffTimestamp = frame->timestamp - this->lastTimestamp;
                    const auto linDelta = diffTransform.translation();
                    const auto linVel = linDelta / diffTimestamp;
                    // compute in two steps to trick Eigen to evaluate
                    const auto angDelta = diffTransform.so3().log();
                    const auto angVel = angDelta / diffTimestamp;

                    odomMsg->twist.twist.linear.x = linVel.x();
                    odomMsg->twist.twist.linear.y = linVel.y();
                    odomMsg->twist.twist.linear.z = linVel.z();
                    odomMsg->twist.twist.angular.x = angVel.x();
                    odomMsg->twist.twist.angular.y = angVel.y();
                    odomMsg->twist.twist.angular.z = angVel.z();

                    // Set covariance matrices
                    const auto relativeTimestamp = frame->timestamp - this->initTimestamp;
                    const auto covarLinScaled = this->covariance_linear * (1.0 + relativeTimestamp * this->covariance_linear_scaling);
                    const auto covarAngScaled = this->covariance_angular * (1.0 + relativeTimestamp * this->covariance_angular_scaling);
                    setCovarianceMatrixFromValues(covarLinScaled, covarAngScaled, odomMsg->pose.covariance);
                    setCovarianceMatrixFromValues(covarLinScaled, covarAngScaled, odomMsg->twist.covariance);
                    // setCovarianceMatrixFromVectors(linDelta * this->covariance_linear, angDelta * this->covariance_angular, odomMsg->pose.covariance);
                    // setCovarianceMatrixFromVectors(linDelta * this->covariance_linear, angDelta * this->covariance_angular, odomMsg->twist.covariance);
                    // setCovarianceMatrixFromTwist(odomMsg->twist.twist, this->covariance_linear, this->covariance_angular, odomMsg->pose.covariance);
                    // setCovarianceMatrixFromTwist(odomMsg->twist.twist, this->covariance_linear, this->covariance_angular, odomMsg->twist.covariance);

                    this->pub_odometry->publish(std::move(odomMsg));
                }

                if (this->publish_tf)
                {
                    geometry_msgs::msg::TransformStamped tf_odom_base;
                    tf_odom_base.header = msg->header;
                    tf_odom_base.child_frame_id = tf2_imu_base_available ? frame_base : frame_imu;
                    tf2::toMsg(tf2_odom_base, tf_odom_base.transform);
                    this->tfbc_odom_base->sendTransform(tf_odom_base);
                }
                if (this->reset_origin)
                {
                    // Publish new origin
                    geometry_msgs::msg::TransformStamped tf_origin_odom;
                    tf_origin_odom.header.stamp = msg->header.stamp;
                    tf_origin_odom.header.frame_id = this->frame_origin;
                    tf_origin_odom.child_frame_id = this->frame_odom;
                    tf2::toMsg(tf2_odom_base.inverse(), tf_origin_odom.transform);
                    this->tfbc_origin_odom->sendTransform(tf_origin_odom);
                    this->reset_origin = false;
                }
            }
            else
            {
                msg->scale = std::numeric_limits<double>::quiet_NaN();
                if (this->transformDSOToIMU)
                    assert(this->transformDSOToIMU->getScale() == 1.0);
            }

            if (this->transformDSOToIMU)
            {
                Sophus::SO3d gravityDirection = this->transformDSOToIMU->getR_dsoW_metricW();
                msg->rotation_metric_to_dso.x = gravityDirection.unit_quaternion().x();
                msg->rotation_metric_to_dso.y = gravityDirection.unit_quaternion().y();
                msg->rotation_metric_to_dso.z = gravityDirection.unit_quaternion().z();
                msg->rotation_metric_to_dso.w = gravityDirection.unit_quaternion().w();
                setPoseFromSE3(this->transformDSOToIMU->getT_cam_imu(), msg->imu_to_cam);
            }
        }

        // Publish visualization
        if (this->pub_keyframes->get_subscription_count() > 0)
        {
            currentCam->setFromF(frame, HCalib);
            auto cam = currentCam->drawCam(msg->header.stamp, nullptr, 0.01, 0.1, 0);
            if (cam)
                this->pub_keyframes->publish(std::move(cam));
        }
        auto &pos = trajectory.points.emplace_back();
        pos.x = msg->pose.position.x;
        pos.y = msg->pose.position.y;
        pos.z = msg->pose.position.z;
        trajectory.header.stamp = msg->header.stamp;
        if (this->pub_trajectory->get_subscription_count() > 0)
            this->pub_trajectory->publish(this->trajectory);

        this->lastTimestamp = frame->timestamp;
        this->lastCamToWorld = camToWorld;
        this->pub_pose_dmvio->publish(std::move(msg));
    }

    void ROS2Wrapper::pushLiveFrame(dso::FrameHessian *image)
    {
        if (this->pub_image_live->get_subscription_count() > 0)
        {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stampFromDSO(image->shell->timestamp);
            msg->header.frame_id = this->frame_camera;
            msg->height = this->dsoSettings.calibG.hG[0];
            msg->width = this->dsoSettings.calibG.wG[0];
            msg->encoding = "mono8";
            msg->is_bigendian = false;
            msg->step = this->dsoSettings.calibG.wG[0];
            msg->data.resize(msg->width * msg->height);

            for (size_t i = 0; i < msg->data.size(); ++i)
            {
                msg->data[i] = image->dI[i][0] * 0.8 > 255.0f ? 255 : static_cast<uint8_t>(image->dI[i][0] * 0.8);
            }

            this->pub_image_live->publish(std::move(msg));
        }
    }

    void ROS2Wrapper::pushDepthImage(dso::MinimalImageB3 *image, dso::FrameHessian *KF)
    {
        if (this->pub_depth_image->get_subscription_count() > 0)
        {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stampFromDSO(KF->shell->timestamp);
            msg->header.frame_id = this->frame_camera;
            msg->height = image->h;
            msg->width = image->w;
            msg->encoding = "bgr8";
            msg->is_bigendian = false;
            msg->step = image->w * 3;
            msg->data.resize(msg->width * msg->height * 3);

            std::memcpy(msg->data.data(), image->data, msg->data.size());

            this->pub_depth_image->publish(std::move(msg));
        }
    }

    void ROS2Wrapper::pushDepthImageFloat(dso::MinimalImageF *image, dso::FrameHessian *KF)
    {
        if (this->pub_depth_float->get_subscription_count() > 0)
        {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stampFromDSO(KF->shell->timestamp);
            msg->header.frame_id = this->frame_camera;
            msg->height = image->h;
            msg->width = image->w;
            msg->encoding = "32FC1";
            msg->is_bigendian = false;
            msg->step = image->w * 4;
            msg->data.resize(msg->width * msg->height * 4);

            std::memcpy(msg->data.data(), image->data, msg->data.size());

            this->pub_depth_float->publish(std::move(msg));
        }
    }

    void ROS2Wrapper::publishKeyframes(std::vector<dso::FrameHessian *> &frames, bool final, dso::CalibHessian *HCalib)
    {
        (void) final;
        static const float color[3] = {0.0f, 0.0f, 1.0f};
        std::unique_lock<std::mutex> lk(mutex);
        const auto stamp = this->get_clock()->now();
        for (dso::FrameHessian *fh : frames)
        {
            if (keyframesByKFID.find(fh->frameID) == keyframesByKFID.end())
            {
                KeyFrameDisplay *kfd = new KeyFrameDisplay(&(this->dsoSettings), "keyframe", this->frame_odom);
                keyframesByKFID[fh->frameID] = kfd;
                keyframes.push_back(kfd);
            }
            auto &kfd = keyframesByKFID[fh->frameID];
            kfd->setFromKF(fh, HCalib);
            if (this->pub_keyframes->get_subscription_count() > 0)
            {
                auto cam = kfd->drawCam(stamp, color);
                if (cam)
                    this->pub_keyframes->publish(std::move(cam));
            }
            if (this->pub_pointcloud->get_subscription_count() > 0)
            {
                auto pointcloud = kfd->drawPointcloud(stamp);
                if (pointcloud)
                    this->pub_pointcloud->publish(std::move(pointcloud));
            }
        }
        if (this->pub_trajectory->get_subscription_count() > 0)
        {
            auto msg = std::make_unique<visualization_msgs::msg::Marker>();
            msg->header.frame_id = this->frame_odom;
            msg->header.stamp = stamp;
            msg->ns = "keyframe";
            msg->id = 0;
            msg->type = visualization_msgs::msg::Marker::LINE_STRIP;
            msg->action = visualization_msgs::msg::Marker::MODIFY;

            msg->color.r = 0.0;
            msg->color.g = 1.0;
            msg->color.b = 0.0;
            msg->color.a = 1.0;
            msg->scale.x = 0.03;

            for (unsigned int i = 0; i < keyframes.size(); i++)
            {
                const auto t = keyframes[i]->camToWorld.translation().cast<float>();
                auto &p = msg->points.emplace_back();
                p.x = t.x();
                p.y = t.y();
                p.z = t.z();
            }
            this->pub_trajectory->publish(std::move(msg));
        }
    }

    void ROS2Wrapper::publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity)
    {
        {
            std::unique_lock<std::mutex> lk(mutex);
            connections.resize(connectivity.size());
            int runningID = 0;
            int totalActFwd = 0, totalActBwd = 0, totalMargFwd = 0, totalMargBwd = 0;
            for (std::pair<uint64_t, Eigen::Vector2i> p : connectivity)
            {
                int host = static_cast<int>(p.first >> 32);
                int target = static_cast<int>(p.first & (uint64_t)0xFFFFFFFF);

                assert(host >= 0 && target >= 0);
                if (host == target)
                {
                    assert(p.second[0] == 0 && p.second[1] == 0);
                    continue;
                }

                if (host > target)
                    continue;

                connections[runningID].from = keyframesByKFID.count(host) == 0 ? 0 : keyframesByKFID[host];
                connections[runningID].to = keyframesByKFID.count(target) == 0 ? 0 : keyframesByKFID[target];
                connections[runningID].fwdAct = p.second[0];
                connections[runningID].fwdMarg = p.second[1];
                totalActFwd += p.second[0];
                totalMargFwd += p.second[1];

                uint64_t inverseKey = (((uint64_t)target) << 32) + ((uint64_t)host);
                Eigen::Vector2i st = connectivity.at(inverseKey);
                connections[runningID].bwdAct = st[0];
                connections[runningID].bwdMarg = st[1];

                totalActBwd += st[0];
                totalMargBwd += st[1];

                runningID++;
            }
        }
        const auto stamp = this->get_clock()->now();
        if (this->pub_constraints->get_subscription_count() > 0)
        {
            auto msg = std::make_unique<visualization_msgs::msg::Marker>();
            msg->header.frame_id = this->frame_odom;
            msg->header.stamp = stamp;
            msg->ns = "all";
            msg->id = 0;
            msg->type = visualization_msgs::msg::Marker::LINE_LIST;
            msg->action = visualization_msgs::msg::Marker::MODIFY;

            msg->color.r = 0.0;
            msg->color.g = 1.0;
            msg->color.b = 0.0;
            msg->color.a = 1.0;
            msg->scale.x = 0.01;

            for (unsigned int i = 0; i < connections.size(); i++)
            {
                if (connections[i].to == 0 || connections[i].from == 0)
                    continue;
                int nAct = connections[i].bwdAct + connections[i].fwdAct;
                int nMarg = connections[i].bwdMarg + connections[i].fwdMarg;
                if (nAct == 0 && nMarg > 0)
                {
                    const auto from = connections[i].from->camToWorld.translation().cast<float>();
                    const auto to = connections[i].to->camToWorld.translation().cast<float>();
                    auto &fp = msg->points.emplace_back();
                    fp.x = from.x();
                    fp.y = from.y();
                    fp.z = from.z();
                    auto &tp = msg->points.emplace_back();
                    tp.x = to.x();
                    tp.y = to.y();
                    tp.z = to.z();
                }
            }
            this->pub_constraints->publish(std::move(msg));
        }
        if (this->pub_constraints->get_subscription_count() > 0)
        {
            auto msg = std::make_unique<visualization_msgs::msg::Marker>();
            msg->header.frame_id = this->frame_odom;
            msg->header.stamp = stamp;
            msg->ns = "active";
            msg->id = 0;
            msg->type = visualization_msgs::msg::Marker::LINE_LIST;
            msg->action = visualization_msgs::msg::Marker::MODIFY;

            msg->color.r = 0.0;
            msg->color.g = 0.0;
            msg->color.b = 1.0;
            msg->color.a = 1.0;
            msg->scale.x = 0.03;

            for (unsigned int i = 0; i < connections.size(); i++)
            {
                if (connections[i].to == 0 || connections[i].from == 0)
                    continue;
                int nAct = connections[i].bwdAct + connections[i].fwdAct;

                if (nAct > 0)
                {
                    const auto from = connections[i].from->camToWorld.translation().cast<float>();
                    const auto to = connections[i].to->camToWorld.translation().cast<float>();
                    auto &fp = msg->points.emplace_back();
                    fp.x = from.x();
                    fp.y = from.y();
                    fp.z = from.z();
                    auto &tp = msg->points.emplace_back();
                    tp.x = to.x();
                    tp.y = to.y();
                    tp.z = to.z();
                }
            }
            this->pub_constraints->publish(std::move(msg));
        }
    }

    void ROS2Wrapper::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg_img)
    {
        double exposure = 0.0;
        if (this->dsoSettings.useExposure)
        {
            try
            {
                exposure = std::stol(msg_img->header.frame_id) * 1e-6;
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "Failed to parse exposure time from frame_id: %s, error: %s", msg_img->header.frame_id.c_str(), e.what());
            }
        }

        const double timestamp = rclcpp::Time(msg_img->header.stamp).seconds() + this->camTimeOffset;
        const auto cv_ptr = cv_bridge::toCvShare(msg_img, sensor_msgs::image_encodings::MONO8);
        const auto minImg = dso::MinimalImageB((int)cv_ptr->image.cols, (int)cv_ptr->image.rows, (unsigned char *)cv_ptr->image.data);
        auto undistImg = std::unique_ptr<dso::ImageAndExposure>(this->undistorter->undistort<unsigned char>(&minImg, exposure, timestamp, 1.0f));

        RCLCPP_NDEBUG(get_logger(), "<callbackImage> stamp=%f\n\t\tmsg\t=\t%p\n\t\tcv\t=\t%p\n\t\tminimg\t=\t%p\n\t\tundist\t=\t%p",
                      rclcpp::Time(msg_img->header.stamp).seconds(), msg_img->data.data(), cv_ptr->image.data, minImg.data, undistImg->image);

        this->imuInt.addImage(std::move(undistImg), timestamp);
    }

    void ROS2Wrapper::callbackIMU(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
    {
        RCLCPP_NDEBUG(get_logger(), "<callbackIMU> stamp=%f address=%p", rclcpp::Time(msg->header.stamp).seconds(), msg.get());

        std::vector<float> accData;
        accData.reserve(3);
        accData.push_back(msg->linear_acceleration.x);
        accData.push_back(msg->linear_acceleration.y);
        accData.push_back(msg->linear_acceleration.z);

        std::vector<float> gyrData;
        gyrData.reserve(3);
        gyrData.push_back(msg->angular_velocity.x);
        gyrData.push_back(msg->angular_velocity.y);
        gyrData.push_back(msg->angular_velocity.z);

        const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        this->imuInt.addAccData(std::move(accData), timestamp);
        this->imuInt.addGyrData(std::move(gyrData), timestamp);
    }

    void ROS2Wrapper::callbackResetOrigin(const std_msgs::msg::Header::ConstSharedPtr &msg)
    {
        this->reset_origin = true;
        RCLCPP_INFO(get_logger(), "Received origin reset request from: %s", msg->frame_id.c_str());
    }

    void ROS2Wrapper::callbackResetOdometry(const std_msgs::msg::Header::ConstSharedPtr &msg)
    {
        this->dsoSettings.fullResetRequested = true;
        RCLCPP_INFO(get_logger(), "Received odmetry reset request from: %s", msg->frame_id.c_str());
    }

    void ROS2Wrapper::run()
    {
        size_t image_id = 0;

        try
        {
            while (!this->stopSystem)
            {
                // Skip the first few frames if the start variable is set.
                if (this->start > 0 && image_id < this->start)
                {
                    auto pair = this->frameContainer.getImageAndIMUData(0);
                    ++image_id;
                    continue;
                }

                const int numSkipFrames = this->frameSkipping->getMaxSkipFrames(this->frameContainer.getQueueSize());
                auto pair = this->frameContainer.getImageAndIMUData(numSkipFrames);

                if (!pair.first)
                    continue;

                this->fullSystem->addActiveFrame(pair.first.get(), image_id, &(pair.second), nullptr);

                if (this->fullSystem->initFailed)
                    RCLCPP_INFO(get_logger(), "Init Failed!");
                if (this->fullSystem->isLost)
                    RCLCPP_INFO(get_logger(), "Lost!");

                if (this->fullSystem->initFailed || this->fullSystem->isLost || this->dsoSettings.fullResetRequested)
                {
                    RCLCPP_INFO(get_logger(), "RESETTING!");
                    this->reset_system();
                    this->dsoSettings.fullResetRequested = false;

                    visualization_msgs::msg::Marker msg;
                    msg.header.frame_id = this->frame_odom;
                    msg.header.stamp = this->get_clock()->now();
                    msg.action = visualization_msgs::msg::Marker::DELETEALL;
                    this->pub_pointcloud->publish(msg);
                    this->pub_keyframes->publish(msg);
                    this->pub_constraints->publish(msg);
                    this->pub_trajectory->publish(msg);
                    this->trajectory.points.clear();
                }

                ++image_id;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception in run loop: %s", e.what());
            // Explicitly stop subscriptions to avoid leaking memory
            this->sub_imu.reset();
            this->sub_img.reset();
        }
        this->fullSystem->blockUntilMappingIsFinished();

        this->fullSystem->printResult(this->imuSettings.resultsPrefix + "result.txt", false, false, true);
        this->fullSystem->printResult(this->imuSettings.resultsPrefix + "resultScaled.txt", false, true, true);

        dmvio::TimeMeasurement::saveResults(this->imuSettings.resultsPrefix + "timings.txt");

        for (dso::IOWrap::Output3DWrapper *ow : this->fullSystem->outputWrapper)
        {
            ow->join();
        }

        this->fullSystem.reset();
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dmvio::ROS2Wrapper)
