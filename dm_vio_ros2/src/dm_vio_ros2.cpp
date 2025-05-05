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
#include "util/FrameShell.h"
#include "util/TimeMeasurement.h"
#include "GTSAMIntegration/PoseTransformationIMU.h"
#include "cv_bridge/cv_bridge.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "basalt/calibration/calibration.hpp"
#include "basalt/serialization/headers_serialization.h"

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

    inline rclcpp::Time stampFromDSO(double timestamp)
    {
        return std::move(rclcpp::Time(timestamp * 1e9));
    }

    void ROS2Wrapper::reset_system()
    {
        this->fullSystem = std::make_unique<dso::FullSystem>(false, this->imuCalibration, this->imuSettings, &dsoSettings);

        if (this->dsoSettings.photometricCalibration > 0 && this->undistorter->photometricUndist == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Photometric calibration not available! Need to use mode=1 or mode=2");
            throw std::invalid_argument("Photometric calibration not available!");
        }

        if (this->undistorter->photometricUndist != nullptr)
        {
            this->fullSystem->setGammaFunction(this->undistorter->photometricUndist->getG());
        }

        this->fullSystem->outputWrapper.push_back(this->frameSkipping.get());
        this->fullSystem->outputWrapper.push_back(this);
    }

    ROS2Wrapper::ROS2Wrapper(const rclcpp::NodeOptions &options)
        : Node("dm_vio", options), syncImage(this->subscriptionImage, this->subscriptionImageInfo, rclcpp::SensorDataQoS().depth()), imuInt(frameContainer, nullptr), mainSettings(&dsoSettings)
    {
        // declare params
        this->declare_parameter("calibration", "");
        this->declare_parameter("camera_index", 0);
        this->declare_parameter("mode", 0);
        this->declare_parameter("preset", 0);
        this->declare_parameter("use_imu", true);
        this->declare_parameter("quiet", true);
        this->declare_parameter("nolog", true);
        this->declare_parameter("results_path", std::filesystem::temp_directory_path() / "dm-vio-results");
        this->declare_parameter("frame_odom", "odom");
        this->declare_parameter("frame_base", "base");
        this->declare_parameter("frame_imu", "imu");
        this->declare_parameter("frame_camera", "camera");
        this->declare_parameter("publish_tf", true);

        // get params
        std::string calib_path = this->get_parameter("calibration").as_string();
        this->dsoSettings.multiCameraIndex = this->get_parameter("camera_index").as_int();
        int mode = this->get_parameter("mode").as_int();
        int preset = this->get_parameter("preset").as_int();
        bool quiet = this->get_parameter("quiet").as_bool();
        bool nolog = this->get_parameter("nolog").as_bool();
        bool use_imu = this->get_parameter("use_imu").as_bool();
        imuSettings.resultsPrefix = std::filesystem::path(this->get_parameter("results_path").as_string()).string() + '/';
        this->frame_odom = this->get_parameter("frame_odom").as_string();
        this->frame_imu = this->get_parameter("frame_imu").as_string();
        this->frame_base = this->get_parameter("frame_base").as_string();
        this->frame_camera = this->get_parameter("frame_camera").as_string();
        this->publishTf = this->get_parameter("publish_tf").as_bool();

        if (calib_path.empty())
            throw std::invalid_argument("Calibration path not set!");

        std::filesystem::create_directory(imuSettings.resultsPrefix);
        if (!std::filesystem::exists(imuSettings.resultsPrefix))
            throw std::invalid_argument("Results path not found!");

        std::ifstream calib_file(calib_path);
        if (!calib_file.good())
            throw std::invalid_argument("Calibration file not found!");

        // apply params
        basalt::Calibration<double> calib;
        cereal::JSONInputArchive archive(calib_file);
        archive(calib);
        camTimeOffset = calib.cam_time_offset_ns * 1e-9;

        switch (mode)
        {
        case 0:
            RCLCPP_INFO(get_logger(), "PHOTOMETRIC MODE WITH CALIBRATION!");
            break;
        case 1:
            RCLCPP_INFO(get_logger(), "PHOTOMETRIC MODE WITHOUT CALIBRATION!");
            this->dsoSettings.photometricCalibration = 0;
            this->dsoSettings.affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            this->dsoSettings.affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            break;
        case 2:
            RCLCPP_INFO(get_logger(), "PHOTOMETRIC MODE WITH PERFECT IMAGES!");
            this->dsoSettings.photometricCalibration = 0;
            this->dsoSettings.affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            this->dsoSettings.affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            this->dsoSettings.minGradHistAdd = 3;
            break;
        case 3:
            // This mode is useful because mode 0 assumes that exposure is available (as it adds a strong prior to
            // the affine brightness change between images), and mode 1 does not use vignette at all.
            // This mode uses vignette (and response), but still fully optimizes brightness changes, hence it is
            // appropriate for sensors without exposure time but with a calibrated vignette.
            RCLCPP_INFO(get_logger(), "PHOTOMETRIC MODE WITH CALIBRATION, BUT NO OR INACCURATE EXPOSURE!");
            this->dsoSettings.affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            this->dsoSettings.affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            break;
        default:
            throw std::invalid_argument("PHOTOMETRIC MODE UNKNOWN!");
        }

        this->mainSettings.settingsDefault(preset);

        if (quiet)
        {
            this->dsoSettings.debugout_runquiet = true;
            RCLCPP_INFO(get_logger(), "QUIET MODE, I'll shut up!");
        }
        if (nolog)
        {
            this->dsoSettings.logStuff = false;
            RCLCPP_INFO(get_logger(), "DISABLE LOGGING!");
        }
        if (use_imu)
        {
            RCLCPP_INFO(get_logger(), "Enabling IMU integration!");
            this->dsoSettings.useIMU = true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Disabling IMU integration!");
            this->dsoSettings.useIMU = false;
        }

        this->undistorter.reset(dso::Undistort::makeFromBasaltCalibration(&dsoSettings, calib_path));

        dso::setGlobalCalib(
            this->undistorter->getSize()[0],
            this->undistorter->getSize()[1],
            this->undistorter->getK().cast<float>(),
            &dsoSettings);

        this->imuCalibration.loadFromFile(calib_path, this->dsoSettings.multiCameraIndex);

        // setup ros
        const auto setup_time = get_clock()->now();

        auto sub_options_imu = rclcpp::SubscriptionOptions();
        sub_options_imu.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options_image = rclcpp::SubscriptionOptions();
        sub_options_image.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        this->subscriptionImage.subscribe(this, "image_raw", rclcpp::SensorDataQoS().get_rmw_qos_profile(), sub_options_image);
        this->subscriptionImageInfo.subscribe(this, "image_info", rclcpp::SensorDataQoS().get_rmw_qos_profile(), sub_options_image);
        this->syncImage.registerCallback(std::bind(&ROS2Wrapper::callbackImage, this, std::placeholders::_1, std::placeholders::_2));
        this->subscriptionIMU = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(), std::bind(&ROS2Wrapper::callbackIMU, this, std::placeholders::_1), sub_options_imu);

        this->systemStatePublisher = this->create_publisher<dm_vio_msgs::msg::DMVIOState>("tracking_state", rclcpp::SensorDataQoS());
        this->dmvioPosePublisher = this->create_publisher<dm_vio_msgs::msg::DMVIOPose>("pose_dmvio", rclcpp::SensorDataQoS());
        this->unscaledPosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_raw", rclcpp::SensorDataQoS());
        // While we publish the metric pose for convenience we don't recommend using it.
        // The reason is that the scale used for generating it might change over time.
        // Usually it is better to save the trajectory and multiply all of it with the newest scale.
        this->metricPosePublisher = this->create_publisher<nav_msgs::msg::Odometry>("pose_metric", rclcpp::SensorDataQoS());

        this->liveImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("image_live", rclcpp::SensorDataQoS());
        this->liveDepthPublisher = this->create_publisher<sensor_msgs::msg::Image>("image_depth", rclcpp::SensorDataQoS());
        this->liveDepthFloatPublisher = this->create_publisher<sensor_msgs::msg::Image>("image_depth_float", rclcpp::SensorDataQoS());

        if (publishTf)
        {
            this->tfsbc_imu_camera = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
            this->tfbc_imu_camera = std::make_unique<tf2_ros::TransformBroadcaster>(this);

            auto tf_imu_camera = geometry_msgs::msg::TransformStamped();
            tf_imu_camera.header.stamp = setup_time;
            tf_imu_camera.header.frame_id = frame_imu;
            tf_imu_camera.child_frame_id = frame_camera;

            setTransformFromSE3(calib.T_i_c[this->dsoSettings.multiCameraIndex], tf_imu_camera.transform);
            tfbc_imu_camera->sendTransform(tf_imu_camera);

            tf_imu_camera.child_frame_id += "_calibration";
            tfsbc_imu_camera->sendTransform(tf_imu_camera);

            this->tfbc_odom_base = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            this->tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            this->tfListener = std::make_unique<tf2_ros::TransformListener>(*this->tfBuffer, this, false);
        }

        auto static_pub_options = rclcpp::PublisherOptions();
        static_pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        this->cameraInfoPublisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::QoS(1).reliable().transient_local(), static_pub_options);
        auto camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
        camera_info->header.stamp = setup_time;
        camera_info->header.frame_id = frame_camera;
        camera_info->width = dso::wG[0];
        camera_info->height = dso::hG[0];
        camera_info->k[0] = dso::KG[0](0, 0);
        camera_info->k[2] = dso::KG[0](0, 2);
        camera_info->k[4] = dso::KG[0](1, 1);
        camera_info->k[5] = dso::KG[0](1, 2);
        camera_info->k[8] = 1.0;
        this->cameraInfoPublisher->publish(std::move(camera_info));

        // setup vio
        this->frameSkipping = std::make_unique<dmvio::FrameSkippingStrategy>(this->frameSkippingSettings);
        this->reset_system();
        this->worker = std::thread(std::bind(&ROS2Wrapper::run, this));
    }

    ROS2Wrapper::~ROS2Wrapper()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down");
        stopSystem = true;
        frameContainer.stop();
        worker.join();
    }

    void ROS2Wrapper::publishTransformDSOToIMU(const TransformDSOToIMU &transformDSOToIMUPassed)
    {
        std::unique_lock<std::mutex> lk(mutex);
        transformDSOToIMU = std::make_unique<dmvio::TransformDSOToIMU>(transformDSOToIMUPassed,
                                                                       std::make_shared<bool>(false),
                                                                       std::make_shared<bool>(false),
                                                                       std::make_shared<bool>(false));
        scaleAvailable = lastSystemStatus == SystemStatus::VISUAL_INERTIAL;
        // You could also publish the new scale (and potentially gravity direction) here already if you want to use it as
        // soon as possible. For this simple ROS wrapper I decided to publish it bundled with the newest tracked pose as
        // this is when it is usually needed.

        //TODO might want to use this
    }

    void ROS2Wrapper::publishSystemStatus(dmvio::SystemStatus systemStatus)
    {
        dm_vio_msgs::msg::DMVIOState msg;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = frame_camera;
        msg.state = static_cast<int>(systemStatus);
        systemStatePublisher->publish(msg);
        lastSystemStatus = systemStatus;
        if (systemStatus == dmvio::SystemStatus::VISUAL_INERTIAL)
        {
            RCLCPP_INFO(get_logger(), "System status: VISUAL_INERTIAL");
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
    }

    void ROS2Wrapper::publishCamPose(dso::FrameShell *frame, dso::CalibHessian *HCalib)
    {
        auto msg = std::make_unique<dm_vio_msgs::msg::DMVIOPose>();
        msg->header.stamp = stampFromDSO(frame->timestamp);
        msg->header.frame_id = frame_odom;

        const auto &camToWorld = frame->camToWorld;
        setPoseFromSE3(camToWorld, msg->pose);

        // Also publish unscaled pose on its own (e.g. for visualization in Rviz).
        auto unscaledMsg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        unscaledMsg->header = msg->header;
        unscaledMsg->pose = msg->pose;
        unscaledPosePublisher->publish(std::move(unscaledMsg));

        {
            std::unique_lock<std::mutex> lk(mutex);
            if (transformDSOToIMU && scaleAvailable)
            {
                msg->scale = transformDSOToIMU->getScale();

                // Transform to metric imu to world. Note that we need to use the inverse as transformDSOToIMU expects
                // worldToCam as an input!
                const auto imuToWorld = Sophus::SE3d(transformDSOToIMU->transformPose(camToWorld.inverse().matrix()));

                if (lastTimestamp > 0)
                {
                    // Publish odometry
                    auto odomMsg = std::make_unique<nav_msgs::msg::Odometry>();
                    odomMsg->header = msg->header;
                    odomMsg->child_frame_id = frame_imu;

                    setPoseFromSE3(imuToWorld, odomMsg->pose.pose);
                    // TODO set covariance

                    // Compute velocity
                    const auto lastImuToWorld = Sophus::SE3d(transformDSOToIMU->transformPose(lastCamToWorld.inverse().matrix()));

                    const auto diffTransform = lastImuToWorld.inverse() * imuToWorld;
                    const auto diffTimestamp = frame->timestamp - lastTimestamp;
                    const auto linVel = diffTransform.translation() / diffTimestamp;
                    const auto angVel = diffTransform.so3().log() / diffTimestamp;

                    odomMsg->twist.twist.linear.x = linVel.x();
                    odomMsg->twist.twist.linear.y = linVel.y();
                    odomMsg->twist.twist.linear.z = linVel.z();
                    odomMsg->twist.twist.angular.x = angVel.x();
                    odomMsg->twist.twist.angular.y = angVel.y();
                    odomMsg->twist.twist.angular.z = angVel.z();
                    // TODO set covariance

                    metricPosePublisher->publish(std::move(odomMsg));
                }

                if (publishTf)
                {
                    try
                    {
                        const auto tf_imu_base = this->tfBuffer->lookupTransform(frame_base, frame_imu, tf2::TimePointZero);

                        tf2::Transform tf2_imu_base, tf2_imu_odom, tf2_odom_base;

                        tf2::fromMsg(tf_imu_base.transform, tf2_imu_base);

                        geometry_msgs::msg::Transform tf_imu_odom;
                        setTransformFromSE3(imuToWorld.inverse(), tf_imu_odom);
                        tf2::fromMsg(tf_imu_odom, tf2_imu_odom);

                        tf2_odom_base = (tf2_imu_base * tf2_imu_odom).inverse();

                        geometry_msgs::msg::TransformStamped tf_odom_base;
                        tf_odom_base.header = msg->header;
                        tf_odom_base.child_frame_id = frame_base;
                        tf2::toMsg(tf2_odom_base, tf_odom_base.transform);
                        this->tfbc_odom_base->sendTransform(tf_odom_base);
                    }
                    catch (const tf2::TransformException &e)
                    {
                        RCLCPP_WARN_ONCE(get_logger(), "Transform exception: %s", e.what());
                    }
                }
            }
            else
            {
                msg->scale = std::numeric_limits<double>::quiet_NaN();
                if (transformDSOToIMU)
                    assert(transformDSOToIMU->getScale() == 1.0);
            }

            if (transformDSOToIMU)
            {
                Sophus::SO3d gravityDirection = transformDSOToIMU->getR_dsoW_metricW();
                msg->rotation_metric_to_dso.x = gravityDirection.unit_quaternion().x();
                msg->rotation_metric_to_dso.y = gravityDirection.unit_quaternion().y();
                msg->rotation_metric_to_dso.z = gravityDirection.unit_quaternion().z();
                msg->rotation_metric_to_dso.w = gravityDirection.unit_quaternion().w();
                setPoseFromSE3(transformDSOToIMU->getT_cam_imu(), msg->imu_to_cam);
            }
        }

        lastTimestamp = frame->timestamp;
        lastCamToWorld = camToWorld;
        dmvioPosePublisher->publish(std::move(msg));
    }

    void ROS2Wrapper::pushLiveFrame(dso::FrameHessian *image)
    {
        if (liveImagePublisher->get_subscription_count() > 0)
        {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stampFromDSO(image->shell->timestamp);
            msg->header.frame_id = frame_camera;
            msg->height = dso::hG[0];
            msg->width = dso::wG[0];
            msg->encoding = "mono8";
            msg->is_bigendian = false;
            msg->step = dso::wG[0];
            msg->data.resize(msg->width * msg->height);

            for (size_t i = 0; i < msg->data.size(); ++i)
            {
                msg->data[i] = image->dI[i][0] * 0.8 > 255.0f ? 255 : static_cast<uint8_t>(image->dI[i][0] * 0.8);
            }

            liveImagePublisher->publish(std::move(msg));
        }
    }

    void ROS2Wrapper::pushDepthImage(dso::MinimalImageB3 *image, dso::FrameHessian *KF)
    {
        if (liveDepthPublisher->get_subscription_count() > 0)
        {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stampFromDSO(KF->shell->timestamp);
            msg->header.frame_id = frame_camera;
            msg->height = image->h;
            msg->width = image->w;
            msg->encoding = "bgr8";
            msg->is_bigendian = false;
            msg->step = image->w * 3;
            msg->data.resize(msg->width * msg->height * 3);

            std::memcpy(msg->data.data(), image->data, msg->data.size());

            liveDepthPublisher->publish(std::move(msg));
        }
    }

    void ROS2Wrapper::pushDepthImageFloat(dso::MinimalImageF *image, dso::FrameHessian *KF)
    {
        if (liveDepthFloatPublisher->get_subscription_count() > 0)
        {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stampFromDSO(KF->shell->timestamp);
            msg->header.frame_id = frame_camera;
            msg->height = image->h;
            msg->width = image->w;
            msg->encoding = "32FC1";
            msg->is_bigendian = false;
            msg->step = image->w * 4;
            msg->data.resize(msg->width * msg->height * 4);

            std::memcpy(msg->data.data(), image->data, msg->data.size());

            liveDepthFloatPublisher->publish(std::move(msg));
        }
    }

    void ROS2Wrapper::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg_img, const image_info_msgs::msg::ImageInfo::ConstSharedPtr &msg_info)
    {
        double timestamp = rclcpp::Time(msg_img->header.stamp).seconds() + camTimeOffset;
        auto cv_ptr = cv_bridge::toCvShare(msg_img, sensor_msgs::image_encodings::MONO8);

        dso::MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows, (unsigned char *)cv_ptr->image.data);
        std::unique_ptr<dso::ImageAndExposure> undistImg(undistorter->undistort<unsigned char>(&minImg, rclcpp::Time(msg_info->exposure).seconds() * 1e3, timestamp, 1.0f));

        imuInt.addImage(std::move(undistImg), timestamp);
    }

    void ROS2Wrapper::callbackIMU(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
    {
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
        imuInt.addAccData(std::move(accData), timestamp);
        imuInt.addGyrData(std::move(gyrData), timestamp);
    }

    void ROS2Wrapper::run()
    {
        size_t image_id = 0;

        while (!stopSystem)
        {
            // Skip the first few frames if the start variable is set.
            if (start > 0 && image_id < start)
            {
                auto pair = frameContainer.getImageAndIMUData(0);
                ++image_id;
                continue;
            }

            int numSkipFrames = frameSkipping->getMaxSkipFrames(frameContainer.getQueueSize());
            auto pair = frameContainer.getImageAndIMUData(numSkipFrames);

            if (!pair.first)
                continue;

            this->fullSystem->addActiveFrame(pair.first.get(), image_id, &(pair.second), nullptr);

            if (this->fullSystem->initFailed)
                RCLCPP_DEBUG(this->get_logger(), "Init Failed!");
            if (this->fullSystem->isLost)
                RCLCPP_DEBUG(this->get_logger(), "Lost!");

            if (this->fullSystem->initFailed || this->fullSystem->isLost || this->dsoSettings.fullResetRequested)
            {
                RCLCPP_INFO(get_logger(), "RESETTING!");
                this->reset_system();
                this->dsoSettings.fullResetRequested = false;
            }

            ++image_id;
        }

        fullSystem->blockUntilMappingIsFinished();

        fullSystem->printResult(imuSettings.resultsPrefix + "result.txt", false, false, true);
        fullSystem->printResult(imuSettings.resultsPrefix + "resultScaled.txt", false, true, true);

        dmvio::TimeMeasurement::saveResults(imuSettings.resultsPrefix + "timings.txt");

        for (dso::IOWrap::Output3DWrapper *ow : fullSystem->outputWrapper)
        {
            ow->join();
        }

        fullSystem.reset();
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dmvio::ROS2Wrapper)
