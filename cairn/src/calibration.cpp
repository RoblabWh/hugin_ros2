#include "cairn/calibration.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <variant>

#include <basalt/camera/pinhole_radtan8_camera.hpp>
#include <basalt/serialization/headers_serialization.h>

namespace cairn {
namespace {

// basalt GenericCamera::getName() values we can represent exactly in cuVSLAM.
constexpr char kPinhole[] = "pinhole";
constexpr char kKb4[] = "kb4";
constexpr char kRadtan8[] = "pinhole-radtan8";

std::string Join(const std::vector<std::string> &items, const char *sep) {
  std::ostringstream os;
  for (size_t i = 0; i < items.size(); ++i) {
    if (i != 0) {
      os << sep;
    }
    os << items[i];
  }
  return os.str();
}

// Human-readable identifier for error messages.
std::string camera_label(const basalt::Calibration<double> &calib, size_t i) {
  std::ostringstream os;
  os << "camera " << i;
  if (i < calib.cam_names.size() && !calib.cam_names[i].empty()) {
    os << " (" << calib.cam_names[i] << ")";
  }
  return os.str();
}

void fill_intrinsics(const basalt::Calibration<double> &calib, size_t i,
                     cuvslam::Camera &camera, const RigOptions &options) {
  const auto &generic = calib.intrinsics[i];
  const std::string model = generic.getName();
  const Eigen::VectorXd p = generic.getParam();

  camera.focal = {static_cast<float>(p[0]), static_cast<float>(p[1])};
  camera.principal = {static_cast<float>(p[2]), static_cast<float>(p[3])};
  camera.size = {calib.resolution[i].x(), calib.resolution[i].y()};

  if (model == kPinhole) {
    camera.distortion.model = cuvslam::Distortion::Model::Pinhole;
    camera.distortion.parameters.clear();
    return;
  }

  if (model == kKb4) {
    camera.distortion.model = cuvslam::Distortion::Model::Fisheye;
    camera.distortion.parameters = {
        static_cast<float>(p[4]), // k1
        static_cast<float>(p[5]), // k2
        static_cast<float>(p[6]), // k3
        static_cast<float>(p[7]), // k4
    };
    return;
  }

  if (model == kRadtan8) {
    camera.distortion.model = cuvslam::Distortion::Model::Polynomial;
    camera.distortion.parameters = {
        static_cast<float>(p[4]),  // k1
        static_cast<float>(p[5]),  // k2
        static_cast<float>(p[6]),  // p1
        static_cast<float>(p[7]),  // p2
        static_cast<float>(p[8]),  // k3
        static_cast<float>(p[9]),  // k4
        static_cast<float>(p[10]), // k5
        static_cast<float>(p[11]), // k6
    };

    const auto &radtan =
        std::get<basalt::PinholeRadtan8Camera<double>>(generic.variant);
    if (radtan.getRpmax() != 0.0) {
      options.on_warning(camera_label(calib, i) +
                         ": rpmax=" + std::to_string(radtan.getRpmax()) +
                         " is dropped; cuVSLAM's Polynomial model has no "
                         "projection-domain limit. Features beyond "
                         "that radius may project incorrectly.");
    }
    return;
  }

  throw std::runtime_error(
      camera_label(calib, i) + ": basalt camera model '" + model +
      "' has no cuVSLAM equivalent. Supported models are: " +
      Join({kPinhole, kKb4, kRadtan8}, ", ") +
      ". Re-run the basalt calibration with one of those models.");
}

void fill_imu(const basalt::Calibration<double> &calib,
              const RigOptions &options, const Sophus::SE3d &rig_from_imu,
              cuvslam::Rig &rig) {
  const auto collapse = [&](const Eigen::Vector3d &v, const char *name) {
    const double lo = v.minCoeff();
    const double hi = v.maxCoeff();
    if (lo > 0.0 && (hi - lo) / lo > 0.1) {
      std::ostringstream os;
      os << "IMU " << name << " is anisotropic (" << v.x() << ", " << v.y()
         << ", " << v.z() << "); using the max " << hi
         << " since cuVSLAM takes a single scalar.";
      options.on_warning(os.str());
    }
    return hi;
  };

  cuvslam::ImuCalibration imu;
  imu.rig_from_imu = sophus_to_cuvslam(rig_from_imu);
  imu.frequency = static_cast<float>(calib.imu_update_rate);

  const double scale = options.imu_noise_scale;
  const auto pick = [&](const std::optional<double> &override_value,
                        double from_calib) {
    return static_cast<float>(override_value.value_or(from_calib * scale));
  };

  imu.gyroscope_noise_density =
      pick(options.gyroscope_noise_density,
           collapse(calib.gyro_noise_std, "gyro_noise_std"));
  imu.gyroscope_random_walk =
      pick(options.gyroscope_random_walk,
           collapse(calib.gyro_bias_std, "gyro_bias_std"));
  imu.accelerometer_noise_density =
      pick(options.accelerometer_noise_density,
           collapse(calib.accel_noise_std, "accel_noise_std"));
  imu.accelerometer_random_walk =
      pick(options.accelerometer_random_walk,
           collapse(calib.accel_bias_std, "accel_bias_std"));

  rig.imus.push_back(imu);
}

inline bool has_imu(const basalt::Calibration<double> &calib) {
  return calib.imu_update_rate > 0.0;
}

void validate_rig(const basalt::Calibration<double> &calib,
                  const cuvslam::Rig &rig) {
  const size_t n = calib.num_cams();
  if (n == 0) {
    throw std::runtime_error("calibration contains no cameras");
  }
  if (n > 32) {
    throw std::runtime_error("calibration contains " + std::to_string(n) +
                             " cameras; cuVSLAM supports at most 32");
  }
  if (calib.T_i_c.size() != n || calib.resolution.size() != n) {
    throw std::runtime_error(
        "inconsistent calibration: " + std::to_string(n) + " intrinsics, " +
        std::to_string(calib.T_i_c.size()) + " extrinsics, " +
        std::to_string(calib.resolution.size()) + " resolutions");
  }

  for (size_t i = 1; i < n; ++i) {
    if (calib.resolution[i] != calib.resolution[0]) {
      std::ostringstream os;
      os << "cuVSLAM requires all cameras to have the same resolution, but "
         << camera_label(calib, i) << " is " << calib.resolution[i].x() << "x"
         << calib.resolution[i].y() << " while " << camera_label(calib, 0)
         << " is " << calib.resolution[0].x() << "x" << calib.resolution[0].y()
         << ".";
      throw std::runtime_error(os.str());
    }
  }

  const auto same_pose = [](const cuvslam::Pose &a, const cuvslam::Pose &b) {
    return a.translation == b.translation && a.rotation == b.rotation;
  };
  for (size_t i = 0; i < rig.cameras.size(); ++i) {
    for (size_t j = i + 1; j < rig.cameras.size(); ++j) {
      if (same_pose(rig.cameras[i].rig_from_camera,
                    rig.cameras[j].rig_from_camera)) {
        throw std::runtime_error(camera_label(calib, i) + " and " +
                                 camera_label(calib, j) +
                                 " have identical extrinsics; cuVSLAM requires "
                                 "distinct camera poses.");
      }
    }
  }
}

cuvslam::Odometry::OdometryMode
resolve_mode(const basalt::Calibration<double> &calib,
             const RigOptions &options, bool imu_enabled) {
  using Mode = cuvslam::Odometry::OdometryMode;
  const size_t n = calib.num_cams();

  Mode mode;
  if (options.mode) {
    mode = *options.mode;

    if (mode == Mode::Inertial && !imu_enabled) {
      throw std::runtime_error("odometry_mode=inertial requires an IMU, but "
                               "the calibration has none "
                               "(imu_update_rate is 0) or use_imu is false");
    }
    if (mode != Mode::Mono && n == 1) {
      throw std::runtime_error("odometry_mode=multicamera, inertial and "
                               "multisensor need at least one stereo pair, but "
                               "the calibration has a single camera. Use mono "
                               "instead.");
    }
    if (mode == Mode::Inertial && n > 2) {
      options.on_warning("odometry_mode=inertial supports a single stereo "
                         "pair, but the calibration has " +
                         std::to_string(n) +
                         " cameras. Use multisensor to fuse an IMU on a "
                         "larger rig.");
    }
    if (mode == Mode::Mono && n > 1) {
      options.on_warning("odometry_mode=mono tracks " + camera_label(calib, 0) +
                         " only; the calibration's other " +
                         std::to_string(n - 1) +
                         " camera(s) are ignored. Leave odometry_mode unset to "
                         "use the whole rig.");
    }
  } else if (n == 1) {
    mode = Mode::Mono;
  } else if (!imu_enabled) {
    mode = Mode::Multicamera;
  } else {
    mode = n == 2 ? Mode::Inertial : Mode::Multisensor;
  }

  if (imu_enabled && mode != Mode::Inertial && mode != Mode::Multisensor) {
    options.on_warning("calibration describes an IMU but the odometry mode "
                       "does not fuse one; IMU messages will be ignored.");
  }

  if (mode == Mode::Multisensor) {
    for (size_t i = 0; i < n; ++i) {
      if (calib.intrinsics[i].getName() != kPinhole) {
        options.on_warning(camera_label(calib, i) + ": camera model '" +
                           calib.intrinsics[i].getName() +
                           "' is not pinhole. Multisensor mode's cuNLS "
                           "solver supports pinhole cameras only; "
                           "tracking may be inaccurate or fail.");
      }
    }
  }

  if (mode != Mode::Mono && !calib.overlaps.empty()) {
    const bool any_overlap =
        std::any_of(calib.overlaps.begin(), calib.overlaps.end(),
                    [](const std::vector<size_t> &o) { return !o.empty(); });
    if (!any_overlap) {
      throw std::runtime_error("calibration declares no overlapping cameras "
                               "(all `overlaps` entries are empty), "
                               "but cuVSLAM needs at least one stereo pair for "
                               "Multicamera, Inertial or "
                               "Multisensor mode.");
    }
  }

  return mode;
}

} // namespace

cuvslam::Pose sophus_to_cuvslam(const Sophus::SE3d &pose) {
  const Eigen::Quaterniond q = pose.unit_quaternion().normalized();
  const Eigen::Vector3d t = pose.translation();

  cuvslam::Pose out;
  out.rotation = {static_cast<float>(q.x()), static_cast<float>(q.y()),
                  static_cast<float>(q.z()), static_cast<float>(q.w())};
  out.translation = {static_cast<float>(t.x()), static_cast<float>(t.y()),
                     static_cast<float>(t.z())};
  return out;
}

basalt::Calibration<double> load_calibration(const std::string &path) {
  std::ifstream is(path);
  if (!is.is_open()) {
    throw std::runtime_error("cannot open calibration file: " + path);
  }

  basalt::Calibration<double> calib;
  try {
    cereal::JSONInputArchive archive(is);
    archive(calib);
  } catch (const std::exception &e) {
    throw std::runtime_error("failed to parse basalt calibration '" + path +
                             "': " + e.what());
  }
  return calib;
}

BuiltRig build_rig(const basalt::Calibration<double> &calib,
                   const RigOptions &options) {
  BuiltRig built;
  const size_t n = calib.num_cams();
  if (n == 0 || calib.T_i_c.size() != n || calib.resolution.size() != n) {
    validate_rig(calib, built.rig);
  }

  // Step 1: place the rig on a frame the calibration knows about.
  //   Imu  -> T_ref_cam[i] = T_i_c[i],                 T_ref_imu = identity
  //   Cam0 -> T_ref_cam[i] = T_i_c[0]^-1 * T_i_c[i],   T_ref_imu = T_i_c[0]^-1
  const Sophus::SE3d ref_from_imu =
      options.has_imu ? Sophus::SE3d{} : calib.T_i_c[0].inverse();

  // Step 2: compose the caller-supplied re-basing (identity unless a base_frame
  // was resolved).
  const Sophus::SE3d rig_from_imu = options.rig_from_reference * ref_from_imu;

  built.rig.cameras.resize(n);
  for (size_t i = 0; i < n; ++i) {
    fill_intrinsics(calib, i, built.rig.cameras[i], options);
    built.rig.cameras[i].rig_from_camera =
        sophus_to_cuvslam(rig_from_imu * calib.T_i_c[i]);
  }

  validate_rig(calib, built.rig);

  const bool imu_enabled = options.use_imu && has_imu(calib);
  built.config.odometry_mode = resolve_mode(calib, options, imu_enabled);

  if (imu_enabled && (built.config.odometry_mode ==
                          cuvslam::Odometry::OdometryMode::Inertial ||
                      built.config.odometry_mode ==
                          cuvslam::Odometry::OdometryMode::Multisensor)) {
    fill_imu(calib, options, rig_from_imu, built.rig);
  }

  built.config.multicam_mode = options.multicam_mode;
  built.config.max_frame_delta_s =
      static_cast<float>(options.image_jitter_threshold_ms / 1000.0);
  built.config.rectified_stereo_camera = false;

  return built;
}

} // namespace cairn
