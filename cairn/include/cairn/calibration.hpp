#ifndef CAIRN__CALIBRATION_HPP_
#define CAIRN__CALIBRATION_HPP_

#include <functional>
#include <optional>
#include <string>

#include <basalt/calibration/calibration.hpp>
#include <cuvslam/cuvslam2.h>
#include <sophus/se3.hpp>

namespace cairn {

struct RigOptions {
  // Pose of the reference frame expressed in the desired rig frame.
  Sophus::SE3d rig_from_reference;

  // Tracking mode. Unset derives it from the rig contents:
  //   1 camera            -> Mono
  //   2 cameras + IMU     -> Inertial
  //   3+ cameras + IMU    -> Multisensor
  //   otherwise           -> Multicamera
  std::optional<cuvslam::Odometry::OdometryMode> mode;

  // Whether the calibration includes an IMU, else the rig is anchored on
  // camera 0 instead.
  bool has_imu = true;

  // Ignore the IMU even when the calibration describes one.
  bool use_imu = true;

  // Multiplier applied to all four IMU noise terms.
  double imu_noise_scale = 1.0;

  // Optional absolute overrides, ignoring noise scale.
  std::optional<double> gyroscope_noise_density;
  std::optional<double> gyroscope_random_walk;
  std::optional<double> accelerometer_noise_density;
  std::optional<double> accelerometer_random_walk;

  double image_jitter_threshold_ms = 34.0;

  cuvslam::Odometry::MulticameraMode multicam_mode =
      cuvslam::Odometry::MulticameraMode::Precision;

  // Called on non-fatal findings.
  std::function<void(const std::string &)> on_warning =
      [](const std::string &) {};
};

struct BuiltRig {
  cuvslam::Rig rig;
  cuvslam::Odometry::Config config;
};

// Read a basalt calibration.json.
basalt::Calibration<double> load_calibration(const std::string &path);

// Convert a basalt calibration into a cuVSLAM rig and odometry config.
BuiltRig build_rig(const basalt::Calibration<double> &calib,
                   const RigOptions &options);

// Sophus SE(3) -> cuvslam::Pose
cuvslam::Pose sophus_to_cuvslam(const Sophus::SE3d &pose);

} // namespace cairn

#endif // CAIRN__CALIBRATION_HPP_
