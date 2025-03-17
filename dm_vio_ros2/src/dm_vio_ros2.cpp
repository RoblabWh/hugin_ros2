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

#include "dm_vio_ros2/dm_vio_ros2.hpp"
#include "dso/util/globalCalib.h"
#include "util/FrameShell.h"
#include "util/TimeMeasurement.h"
#include "GTSAMIntegration/PoseTransformationIMU.h"
#include "cv_bridge/cv_bridge.h"
#include "filesystem"

namespace dmvio
{

    ROS2Wrapper::ROS2Wrapper(const rclcpp::NodeOptions &options)
        : Node("dm_vio", options), syncImage(this->subscriptionImage, this->subscriptionImageInfo, 10), imuInt(frameContainer, nullptr)
    {
        this->subscriptionImage.subscribe(this, "image_raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        this->subscriptionImageInfo.subscribe(this, "image_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        this->syncImage.registerCallback(std::bind(&ROS2Wrapper::callbackImage, this, std::placeholders::_1, std::placeholders::_2));
        this->subscriptionIMU = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(), std::bind(&ROS2Wrapper::callbackIMU, this, std::placeholders::_1));

        this->systemStatePublisher = this->create_publisher<dm_vio_msgs::msg::DMVIOState>("tracking_state", rclcpp::SensorDataQoS());
        this->dmvioPosePublisher = this->create_publisher<dm_vio_msgs::msg::DMVIOPose>("pose_dmvio", rclcpp::SensorDataQoS());
        this->unscaledPosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_raw", rclcpp::SensorDataQoS());
        // While we publish the metric pose for convenience we don't recommend using it.
        // The reason is that the scale used for generating it might change over time.
        // Usually it is better to save the trajectory and multiply all of it with the newest scale.
        this->metricPosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_metric", rclcpp::SensorDataQoS());

        // declare params
        this->declare_parameter("calibration", "");
        this->declare_parameter("camera_index", 0);
        this->declare_parameter("mode", 0);
        this->declare_parameter("preset", 0);
        this->declare_parameter("use_imu", true);
        this->declare_parameter("quiet", true);
        this->declare_parameter("nolog", true);
        this->declare_parameter("results_path", std::filesystem::temp_directory_path() / "dm-vio-results");

        // get params
        std::string calib_path = this->get_parameter("calibration").as_string();
        dso::multiCameraIndex = this->get_parameter("camera_index").as_int();
        int mode = this->get_parameter("mode").as_int();
        int preset = this->get_parameter("preset").as_int();
        bool quiet = this->get_parameter("quiet").as_bool();
        bool nolog = this->get_parameter("nolog").as_bool();
        bool use_imu = this->get_parameter("use_imu").as_bool();
        imuSettings.resultsPrefix = std::filesystem::path(this->get_parameter("results_path").as_string()).string() + '/';

        if (calib_path.empty())
            throw std::invalid_argument("Calibration path not set!");

        std::filesystem::create_directory(imuSettings.resultsPrefix);
        if (!std::filesystem::exists(imuSettings.resultsPrefix))
            throw std::invalid_argument("Results path not found!");

        // apply params
        switch (mode)
        {
        case 0:
            RCLCPP_INFO(get_logger(), "PHOTOMETRIC MODE WITH CALIBRATION!");
            break;
        case 1:
            RCLCPP_INFO(get_logger(), "PHOTOMETRIC MODE WITHOUT CALIBRATION!");
            dso::setting_photometricCalibration = 0;
            dso::setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            dso::setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            break;
        case 2:
            RCLCPP_INFO(get_logger(), "PHOTOMETRIC MODE WITH PERFECT IMAGES!");
            dso::setting_photometricCalibration = 0;
            dso::setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            dso::setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            dso::setting_minGradHistAdd = 3;
            break;
        case 3:
            // This mode is useful because mode 0 assumes that exposure is available (as it adds a strong prior to
            // the affine brightness change between images), and mode 1 does not use vignette at all.
            // This mode uses vignette (and response), but still fully optimizes brightness changes, hence it is
            // appropriate for sensors without exposure time but with a calibrated vignette.
            RCLCPP_INFO(get_logger(), "PHOTOMETRIC MODE WITH CALIBRATION, BUT NO OR INACCURATE EXPOSURE!");
            dso::setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            dso::setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            break;
        default:
            throw std::invalid_argument("PHOTOMETRIC MODE UNKNOWN!");
        }

        this->mainSettings.settingsDefault(preset);

        if (quiet)
        {
            dso::setting_debugout_runquiet = true;
            RCLCPP_INFO(get_logger(), "QUIET MODE, I'll shut up!");
        }
        if (nolog)
        {
            dso::setting_logStuff = false;
            RCLCPP_INFO(get_logger(), "DISABLE LOGGING!");
        }
        if (use_imu)
        {
            RCLCPP_INFO(get_logger(), "Enabling IMU integration!");
            dso::setting_useIMU = true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Disabling IMU integration!");
            dso::setting_useIMU = false;
        }

        this->undistorter.reset(dso::Undistort::makeFromBasaltCalibration(calib_path));

        dso::setGlobalCalib(
            this->undistorter->getSize()[0],
            this->undistorter->getSize()[1],
            this->undistorter->getK().cast<float>());

        this->imuCalibration.loadFromFile(calib_path);

        // setup system
        this->frameSkipping = std::make_unique<dmvio::FrameSkippingStrategy>(this->frameSkippingSettings);

        this->fullSystem = std::make_unique<dso::FullSystem>(false, this->imuCalibration, this->imuSettings);

        if (dso::setting_photometricCalibration > 0 && this->undistorter->photometricUndist == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Photometric calibration not available! Need to use mode=1 or mode=2");
            throw std::invalid_argument("Photometric calibration not available!");
        }

        if (this->undistorter->photometricUndist != nullptr)
        {
            fullSystem->setGammaFunction(this->undistorter->photometricUndist->getG());
        }

        this->fullSystem->outputWrapper.push_back(this->frameSkipping.get());
        this->fullSystem->outputWrapper.push_back(this);

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
        msg.header.frame_id = frame_world;
        msg.state = static_cast<int>(systemStatus);
        systemStatePublisher->publish(msg);
        lastSystemStatus = systemStatus;
    }

    void setMsgFromSE3(geometry_msgs::msg::Pose &poseMsg, const Sophus::SE3d &pose)
    {
        poseMsg.position.x = pose.translation()[0];
        poseMsg.position.y = pose.translation()[1];
        poseMsg.position.z = pose.translation()[2];
        poseMsg.orientation.x = pose.so3().unit_quaternion().x();
        poseMsg.orientation.y = pose.so3().unit_quaternion().y();
        poseMsg.orientation.z = pose.so3().unit_quaternion().z();
        poseMsg.orientation.w = pose.so3().unit_quaternion().w();
    }

    void ROS2Wrapper::publishCamPose(dso::FrameShell *frame, dso::CalibHessian *HCalib)
    {
        dm_vio_msgs::msg::DMVIOPose msg;
        msg.header.stamp = rclcpp::Time(frame->timestamp * 1e9);
        msg.header.frame_id = frame_world;

        auto &camToWorld = frame->camToWorld;

        geometry_msgs::msg::Pose &poseMsg = msg.pose;
        setMsgFromSE3(poseMsg, camToWorld);

        // Also publish unscaled pose on its own (e.g. for visualization in Rviz).
        geometry_msgs::msg::PoseStamped unscaledMsg;
        unscaledMsg.header = msg.header;
        unscaledMsg.pose = poseMsg;
        unscaledPosePublisher->publish(unscaledMsg);

        {
            std::unique_lock<std::mutex> lk(mutex);
            if (transformDSOToIMU && scaleAvailable)
            {
                msg.scale = transformDSOToIMU->getScale();

                // Publish scaled pose.
                geometry_msgs::msg::PoseStamped scaledMsg;
                scaledMsg.header = msg.header;

                // Transform to metric imu to world. Note that we need to use the inverse as transformDSOToIMU expects
                // worldToCam as an input!
                Sophus::SE3d imuToWorld(transformDSOToIMU->transformPose(camToWorld.inverse().matrix()));
                setMsgFromSE3(scaledMsg.pose, imuToWorld);

                metricPosePublisher->publish(scaledMsg);
            }
            else
            {
                msg.scale = std::numeric_limits<double>::quiet_NaN();
                if (transformDSOToIMU)
                    assert(transformDSOToIMU->getScale() == 1.0);
            }

            if (transformDSOToIMU)
            {
                Sophus::SO3d gravityDirection = transformDSOToIMU->getR_dsoW_metricW();
                msg.rotation_metric_to_dso.x = gravityDirection.unit_quaternion().x();
                msg.rotation_metric_to_dso.y = gravityDirection.unit_quaternion().y();
                msg.rotation_metric_to_dso.z = gravityDirection.unit_quaternion().z();
                msg.rotation_metric_to_dso.w = gravityDirection.unit_quaternion().w();
                setMsgFromSE3(msg.imu_to_cam, transformDSOToIMU->getT_cam_imu());
            }
        }

        dmvioPosePublisher->publish(msg);
    }

    void ROS2Wrapper::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg_img, const image_info_msgs::msg::ImageInfo::ConstSharedPtr &msg_info)
    {
        double timestamp = rclcpp::Time(msg_img->header.stamp).seconds() + camTimeOffset;
        auto cv_ptr = cv_bridge::toCvShare(msg_img, sensor_msgs::image_encodings::MONO8);
        assert(cv_ptr->image.type() == CV_8U);
        assert(cv_ptr->image.channels() == 1);

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

            if (this->fullSystem->initFailed || this->fullSystem->isLost || dso::setting_fullResetRequested)
            {
                RCLCPP_INFO(get_logger(), "RESETTING!");
                std::vector<dso::IOWrap::Output3DWrapper *> wraps = this->fullSystem->outputWrapper;
                this->fullSystem.reset();
                for (dso::IOWrap::Output3DWrapper *ow : wraps)
                    ow->reset();

                this->fullSystem = std::make_unique<dso::FullSystem>(false, this->imuCalibration, this->imuSettings);
                if (this->undistorter->photometricUndist != nullptr)
                    this->fullSystem->setGammaFunction(this->undistorter->photometricUndist->getG());
                this->fullSystem->outputWrapper = wraps;

                dso::setting_fullResetRequested = false;
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
