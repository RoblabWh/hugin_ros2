#include <string>

#include <gtest/gtest.h>

#include "cairn/calibration.hpp"

namespace {

std::string fixture(const std::string &name) {
  return std::string(CAIRN_TEST_DATA_DIR) + "/" + name;
}

basalt::Calibration<double> load(const std::string &name) {
  return cairn::load_calibration(fixture(name));
}

// Matches the intent of Sophus::SE3d -> cuvslam::Pose without depending on
// float exactness.
void expect_pose_near(const cuvslam::Pose &actual, const Sophus::SE3d &expected,
                      double tol = 1e-6) {
  Eigen::Quaterniond q = expected.unit_quaternion().normalized();
  // Quaternions q and -q are the same rotation, align signs before comparing.
  if ((q.w() < 0) != (actual.rotation[3] < 0)) {
    q.coeffs() *= -1;
  }
  EXPECT_NEAR(actual.rotation[0], q.x(), tol);
  EXPECT_NEAR(actual.rotation[1], q.y(), tol);
  EXPECT_NEAR(actual.rotation[2], q.z(), tol);
  EXPECT_NEAR(actual.rotation[3], q.w(), tol);
  EXPECT_NEAR(actual.translation[0], expected.translation().x(), tol);
  EXPECT_NEAR(actual.translation[1], expected.translation().y(), tol);
  EXPECT_NEAR(actual.translation[2], expected.translation().z(), tol);
}

} // namespace

TEST(Calibration, LoadMissingFileThrows) {
  EXPECT_THROW(cairn::load_calibration("/nonexistent/calib.json"),
               std::runtime_error);
}

TEST(Calibration, Kb4MapsToFisheyeVerbatim) {
  const auto calib = load("kb4_stereo.json");
  const auto built = cairn::build_rig(calib, {});

  ASSERT_EQ(built.rig.cameras.size(), 2u);
  const auto &cam = built.rig.cameras[0];

  EXPECT_EQ(cam.distortion.model, cuvslam::Distortion::Model::Fisheye);
  ASSERT_EQ(cam.distortion.parameters.size(), 4u);
  EXPECT_FLOAT_EQ(cam.distortion.parameters[0], 0.004927917950706341f);
  EXPECT_FLOAT_EQ(cam.distortion.parameters[1], -0.0029907424678906804f);
  EXPECT_FLOAT_EQ(cam.distortion.parameters[2], 0.0005062300598753976f);
  EXPECT_FLOAT_EQ(cam.distortion.parameters[3], -0.0003247362175150025f);

  EXPECT_FLOAT_EQ(cam.focal[0], 191.28012055683033f);
  EXPECT_FLOAT_EQ(cam.focal[1], 191.25382350280705f);
  EXPECT_FLOAT_EQ(cam.principal[0], 254.9379225603446f);
  EXPECT_FLOAT_EQ(cam.principal[1], 256.87495352169867f);
  EXPECT_EQ(cam.size[0], 512);
  EXPECT_EQ(cam.size[1], 512);
}

TEST(Calibration, Radtan8MapsToPolynomialInCuvslamOrder) {
  const auto calib = load("radtan8_mono.json");
  const auto built = cairn::build_rig(calib, {});

  ASSERT_EQ(built.rig.cameras.size(), 1u);
  const auto &cam = built.rig.cameras[0];

  EXPECT_EQ(cam.distortion.model, cuvslam::Distortion::Model::Polynomial);
  ASSERT_EQ(cam.distortion.parameters.size(), 8u);
  EXPECT_FLOAT_EQ(cam.distortion.parameters[0], 0.061508219772028305f);   // k1
  EXPECT_FLOAT_EQ(cam.distortion.parameters[1], -0.10889344366626673f);   // k2
  EXPECT_FLOAT_EQ(cam.distortion.parameters[2], -9.695921542417555e-05f); // p1
  EXPECT_FLOAT_EQ(cam.distortion.parameters[3], -0.004319435470349197f);  // p2
  EXPECT_FLOAT_EQ(cam.distortion.parameters[4], 0.0f);                    // k3
  EXPECT_FLOAT_EQ(cam.distortion.parameters[7], 0.0f);                    // k6
}

TEST(Calibration, PinholeHasNoDistortionParameters) {
  const auto calib = load("pinhole_stereo.json");
  const auto built = cairn::build_rig(calib, {});

  ASSERT_EQ(built.rig.cameras.size(), 2u);
  EXPECT_EQ(built.rig.cameras[0].distortion.model,
            cuvslam::Distortion::Model::Pinhole);
  EXPECT_TRUE(built.rig.cameras[0].distortion.parameters.empty());
}

TEST(Calibration, UnsupportedModelsThrowWithContext) {
  for (const char *fixture : {"eucm_stereo.json", "ds_stereo.json"}) {
    const auto calib = load(fixture);
    try {
      cairn::build_rig(calib, {});
      ADD_FAILURE() << fixture << " should have thrown";
    } catch (const std::runtime_error &e) {
      const std::string what = e.what();
      // The message has to name the camera and the offending model, or it is
      // useless in a log.
      EXPECT_NE(what.find("camera 0"), std::string::npos) << what;
      EXPECT_NE(what.find("no cuVSLAM equivalent"), std::string::npos) << what;
      EXPECT_NE(what.find("kb4"), std::string::npos) << what;
    }
  }
}

TEST(Calibration, MismatchedResolutionsThrow) {
  const auto calib = load("mismatched_resolution.json");
  EXPECT_THROW(cairn::build_rig(calib, {}), std::runtime_error);
}

TEST(Calibration, HasImuAnchorsOnImuAsIdentityConversion) {
  const auto calib = load("kb4_stereo.json");
  cairn::RigOptions options;
  options.has_imu = true;
  const auto built = cairn::build_rig(calib, options);

  for (size_t i = 0; i < 2; ++i) {
    expect_pose_near(built.rig.cameras[i].rig_from_camera, calib.T_i_c[i]);
  }
  ASSERT_EQ(built.rig.imus.size(), 1u);
  expect_pose_near(built.rig.imus[0].rig_from_imu, Sophus::SE3d{});
}

TEST(Calibration, NoImuRebasesOntoFirstCamera) {
  const auto calib = load("kb4_stereo.json");
  cairn::RigOptions options;
  options.has_imu = false;
  const auto built = cairn::build_rig(calib, options);

  expect_pose_near(built.rig.cameras[0].rig_from_camera, Sophus::SE3d{});
  expect_pose_near(built.rig.cameras[1].rig_from_camera,
                   calib.T_i_c[0].inverse() * calib.T_i_c[1]);
  ASSERT_EQ(built.rig.imus.size(), 1u);
  expect_pose_near(built.rig.imus[0].rig_from_imu, calib.T_i_c[0].inverse());
}

TEST(Calibration, RigFromReferenceComposesOntoCamerasAndImu) {
  const auto calib = load("kb4_stereo.json");

  cairn::RigOptions options;
  options.has_imu = true;
  options.rig_from_reference = Sophus::SE3d(
      Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d(0.1, -0.2, 0.05));

  const auto built = cairn::build_rig(calib, options);
  for (size_t i = 0; i < 2; ++i) {
    expect_pose_near(built.rig.cameras[i].rig_from_camera,
                     options.rig_from_reference * calib.T_i_c[i]);
  }
  ASSERT_EQ(built.rig.imus.size(), 1u);
  expect_pose_near(built.rig.imus[0].rig_from_imu, options.rig_from_reference);
}

TEST(Calibration, ImuNoiseUsesMaxAcrossAxes) {
  const auto calib = load("kb4_stereo.json");

  std::vector<std::string> warnings;
  cairn::RigOptions options;
  options.on_warning = [&warnings](const std::string &w) {
    warnings.push_back(w);
  };
  const auto built = cairn::build_rig(calib, options);

  ASSERT_EQ(built.rig.imus.size(), 1u);
  const auto &imu = built.rig.imus[0];
  EXPECT_FLOAT_EQ(imu.gyroscope_noise_density,
                  static_cast<float>(calib.gyro_noise_std.maxCoeff()));
  EXPECT_FLOAT_EQ(imu.accelerometer_noise_density,
                  static_cast<float>(calib.accel_noise_std.maxCoeff()));
  EXPECT_FLOAT_EQ(imu.gyroscope_random_walk,
                  static_cast<float>(calib.gyro_bias_std.maxCoeff()));
  EXPECT_FLOAT_EQ(imu.accelerometer_random_walk,
                  static_cast<float>(calib.accel_bias_std.maxCoeff()));
  EXPECT_FLOAT_EQ(imu.frequency, static_cast<float>(calib.imu_update_rate));

  // These axes differ by well over 10%, so the collapse must be reported.
  EXPECT_FALSE(warnings.empty());
}

TEST(Calibration, ImuNoiseScaleAndOverrides) {
  const auto calib = load("kb4_stereo.json");

  cairn::RigOptions scaled;
  scaled.imu_noise_scale = 4.0;
  const auto built_scaled = cairn::build_rig(calib, scaled);
  ASSERT_EQ(built_scaled.rig.imus.size(), 1u);
  EXPECT_FLOAT_EQ(built_scaled.rig.imus[0].gyroscope_noise_density,
                  static_cast<float>(calib.gyro_noise_std.maxCoeff() * 4.0));

  cairn::RigOptions overridden;
  overridden.imu_noise_scale = 4.0;
  overridden.gyroscope_noise_density = 0.5;
  const auto built_override = cairn::build_rig(calib, overridden);
  EXPECT_FLOAT_EQ(built_override.rig.imus[0].gyroscope_noise_density, 0.5f);
}

TEST(Calibration, ModeSelection) {
  using cairn::build_rig;
  using Mode = cuvslam::Odometry::OdometryMode;

  // 1 camera -> Mono, even though the calibration also describes an IMU:
  // cuVSLAM's Inertial mode wants a stereo pair.
  EXPECT_EQ(build_rig(load("radtan8_mono.json"), {}).config.odometry_mode,
            Mode::Mono);

  // 2 cameras + IMU -> Inertial.
  EXPECT_EQ(build_rig(load("kb4_stereo.json"), {}).config.odometry_mode,
            Mode::Inertial);

  // Same rig with the IMU disabled -> Multicamera, and no IMU in the rig.
  cairn::RigOptions no_imu;
  no_imu.use_imu = false;
  const auto built = build_rig(load("kb4_stereo.json"), no_imu);
  EXPECT_EQ(built.config.odometry_mode, Mode::Multicamera);
  EXPECT_TRUE(built.rig.imus.empty());

  // 3 cameras + IMU -> Multisensor, which is the only mode that fuses an IMU on
  // a rig this size.
  const auto triple = build_rig(load("pinhole_triple.json"), {});
  EXPECT_EQ(triple.config.odometry_mode, Mode::Multisensor);
  EXPECT_EQ(triple.rig.imus.size(), 1u);

  // Same rig without the IMU has nothing to fuse -> Multicamera.
  const auto triple_no_imu = build_rig(load("pinhole_triple.json"), no_imu);
  EXPECT_EQ(triple_no_imu.config.odometry_mode, Mode::Multicamera);
  EXPECT_TRUE(triple_no_imu.rig.imus.empty());
}

TEST(Calibration, CalibrationWithoutImuBuildsCameraOnlyRig) {
  const auto built = cairn::build_rig(load("pinhole_stereo_no_imu.json"), {});
  EXPECT_TRUE(built.rig.imus.empty());
  EXPECT_EQ(built.config.odometry_mode,
            cuvslam::Odometry::OdometryMode::Multicamera);
}

TEST(Calibration, ExplicitInertialWithoutImuThrows) {
  cairn::RigOptions options;
  options.mode = cuvslam::Odometry::OdometryMode::Inertial;
  options.use_imu = false;
  EXPECT_THROW(cairn::build_rig(load("kb4_stereo.json"), options),
               std::runtime_error);
}

TEST(Calibration, ExplicitModeIsHonoured) {
  // Auto would pick Inertial for this rig (2 cameras + IMU), but an explicit
  // mode has to win, or the parameter does nothing.
  cairn::RigOptions options;
  options.mode = cuvslam::Odometry::OdometryMode::Multicamera;

  const auto built = cairn::build_rig(load("kb4_stereo.json"), options);
  EXPECT_EQ(built.config.odometry_mode,
            cuvslam::Odometry::OdometryMode::Multicamera);
  // Multicamera does not fuse an IMU, so the rig must not carry one.
  EXPECT_TRUE(built.rig.imus.empty());
}

TEST(Calibration, ExplicitMonoOnMultiCameraRigWarnsButBuilds) {
  std::vector<std::string> warnings;
  cairn::RigOptions options;
  options.mode = cuvslam::Odometry::OdometryMode::Mono;
  options.on_warning = [&warnings](const std::string &w) {
    warnings.push_back(w);
  };

  const auto built = cairn::build_rig(load("kb4_stereo.json"), options);
  EXPECT_EQ(built.config.odometry_mode, cuvslam::Odometry::OdometryMode::Mono);

  size_t mono_warnings = 0;
  for (const auto &warning : warnings) {
    if (warning.find("odometry_mode=mono") != std::string::npos) {
      ++mono_warnings;
    }
  }
  EXPECT_EQ(mono_warnings, 1u);
}

TEST(Calibration, MultisensorWithoutImuBuilds) {
  std::vector<std::string> warnings;
  cairn::RigOptions options;
  options.mode = cuvslam::Odometry::OdometryMode::Multisensor;
  options.use_imu = false;
  options.on_warning = [&warnings](const std::string &w) {
    warnings.push_back(w);
  };

  const auto built = cairn::build_rig(load("pinhole_triple.json"), options);
  EXPECT_EQ(built.config.odometry_mode,
            cuvslam::Odometry::OdometryMode::Multisensor);
  EXPECT_TRUE(built.rig.imus.empty());
  // All three cameras are pinhole, so nothing to complain about.
  EXPECT_TRUE(warnings.empty());
}

TEST(Calibration, MultisensorWarnsOnNonPinhole) {
  std::vector<std::string> warnings;
  cairn::RigOptions options;
  options.mode = cuvslam::Odometry::OdometryMode::Multisensor;
  options.on_warning = [&warnings](const std::string &w) {
    warnings.push_back(w);
  };

  const auto built = cairn::build_rig(load("kb4_stereo.json"), options);
  EXPECT_EQ(built.config.odometry_mode,
            cuvslam::Odometry::OdometryMode::Multisensor);

  size_t pinhole_warnings = 0;
  for (const auto &warning : warnings) {
    if (warning.find("pinhole") != std::string::npos) {
      ++pinhole_warnings;
    }
  }
  // one per kb4 camera
  EXPECT_EQ(pinhole_warnings, 2u);
}

TEST(Calibration, MaxFrameDeltaFollowsJitterThreshold) {
  cairn::RigOptions options;
  options.image_jitter_threshold_ms = 50.0;
  const auto built = cairn::build_rig(load("kb4_stereo.json"), options);
  EXPECT_FLOAT_EQ(built.config.max_frame_delta_s, 0.05f);
}
