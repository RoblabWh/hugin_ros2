#include "cairn/tracker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace cairn {
namespace {

rclcpp::QoS parse_qos(const std::string &name, size_t depth = 10) {
  if (name == "SENSOR_DATA") {
    return rclcpp::SensorDataQoS();
  }
  if (name == "SYSTEM_DEFAULT") {
    return rclcpp::SystemDefaultsQoS();
  }
  if (name == "DEFAULT") {
    return rclcpp::QoS(depth);
  }
  throw std::runtime_error(
      "unknown QoS profile '" + name +
      "'; expected SENSOR_DATA, DEFAULT or SYSTEM_DEFAULT");
}

Sophus::SE3d cuvslam_to_sophus(const cuvslam::Pose &pose) {
  const Eigen::Quaterniond q(pose.rotation[3], pose.rotation[0],
                             pose.rotation[1], pose.rotation[2]); // w, x, y, z
  const Eigen::Vector3d t(pose.translation[0], pose.translation[1],
                          pose.translation[2]);
  return Sophus::SE3d(q.normalized(), t);
}

geometry_msgs::msg::Pose to_pose_msg(const Sophus::SE3d &pose) {
  geometry_msgs::msg::Pose out;
  const Eigen::Quaterniond q = pose.unit_quaternion();
  out.position.x = pose.translation().x();
  out.position.y = pose.translation().y();
  out.position.z = pose.translation().z();
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
  return out;
}

geometry_msgs::msg::Transform to_transform_msgs(const Sophus::SE3d &pose) {
  geometry_msgs::msg::Transform out;
  const Eigen::Quaterniond q = pose.unit_quaternion();
  out.translation.x = pose.translation().x();
  out.translation.y = pose.translation().y();
  out.translation.z = pose.translation().z();
  out.rotation.x = q.x();
  out.rotation.y = q.y();
  out.rotation.z = q.z();
  out.rotation.w = q.w();
  return out;
}

template <typename PublisherT>
bool has_subscribers(const PublisherT &publisher) {
  return publisher && publisher->get_subscription_count() > 0;
}

bool is_due(int64_t &last_ns, int64_t now_ns, int64_t period_ns) {
  const int64_t elapsed = now_ns - last_ns;
  if (elapsed >= 0 && elapsed < period_ns) {
    return false;
  }
  last_ns = now_ns;
  return true;
}

// Block until the first message arrives on topic and return its
// frame_id.
template <typename MsgT>
std::string wait_for_frame_id(rclcpp::Node &node, const std::string &topic,
                              const rclcpp::QoS &qos) {
  constexpr std::chrono::seconds kLogInterval(5);

  // Intra-process must be OFF for this subscription.
  rclcpp::SubscriptionOptions options;
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  auto subscription = node.create_subscription<MsgT>(
      topic, qos, [](const std::shared_ptr<const MsgT>) {}, options);

  auto context = node.get_node_base_interface()->get_context();
  RCLCPP_INFO(
      node.get_logger(),
      "waiting for the first message on '%s' to learn the reference frame",
      topic.c_str());

  MsgT message;
  while (!rclcpp::wait_for_message<MsgT>(message, subscription, context,
                                         kLogInterval)) {
    if (!rclcpp::ok(context)) {
      throw std::runtime_error(
          "shut down while waiting for the first message on '" + topic + "'");
    }
    RCLCPP_INFO(node.get_logger(),
                "still waiting for the first message on '%s' to learn the "
                "reference frame; set the "
                "'reference_frame' parameter to skip this wait",
                topic.c_str());
  }

  if (message.header.frame_id.empty()) {
    throw std::runtime_error(
        "messages on '" + topic +
        "' carry an empty header.frame_id, so the reference frame "
        "cannot be read from them; fix the publisher or set 'reference_frame' "
        "explicitly");
  }
  return message.header.frame_id;
}

// Deterministic full-saturation colour per observation id.
foxglove_msgs::msg::Color color_from_id(uint64_t id) {
  constexpr uint64_t kSteps = 1000;
  const double hue =
      static_cast<double>((id * 2654435761ull) % kSteps) / kSteps;
  // HSV -> RGB at s = v = 1, as three phase-shifted triangle waves.
  const auto channel = [](double t) {
    t -= std::floor(t);
    return std::clamp(std::abs(t * 6.0 - 3.0) - 1.0, 0.0, 1.0);
  };

  foxglove_msgs::msg::Color color;
  color.r = channel(hue);
  color.g = channel(hue + 2.0 / 3.0);
  color.b = channel(hue + 1.0 / 3.0);
  color.a = 1.0;
  return color;
}

} // namespace

Tracker::Tracker(const rclcpp::NodeOptions &options)
    : rclcpp::Node("cairn", options) {
  const std::string calibration_file =
      declare_parameter<std::string>("calibration_file", "");
  if (calibration_file.empty()) {
    throw std::runtime_error("parameter 'calibration_file' is required: path "
                             "to a basalt calibration.json");
  }

  cuvslam::SetVerbosity(
      static_cast<int>(declare_parameter<int64_t>("verbosity", 0)));

  calib_ = load_calibration(calibration_file);
  num_cameras_ = calib_.num_cams();
  if (num_cameras_ == 0) {
    throw std::runtime_error("calibration contains no cameras");
  }
  RCLCPP_INFO(get_logger(),
              "Loaded %s: %zu camera(s), imu_update_rate=%.1f Hz, "
              "cam_time_offset=%ld ns",
              calibration_file.c_str(), num_cameras_, calib_.imu_update_rate,
              static_cast<long>(calib_.cam_time_offset_ns));

  RigOptions rig_options = readRigOptions();

  std::vector<std::string> image_topics;
  image_topics.reserve(num_cameras_);
  for (size_t i = 0; i < num_cameras_; ++i) {
    image_topics.push_back(i < calib_.cam_names.size() &&
                                   !calib_.cam_names[i].empty()
                               ? calib_.cam_names[i]
                               : "cam" + std::to_string(i) + "/image_raw");
  }
  const auto image_qos =
      parse_qos(declare_parameter<std::string>("image_qos", "SENSOR_DATA"));

  const std::string imu_topic =
      calib_.imu_name.empty() ? "imu" : calib_.imu_name;
  const auto imu_qos =
      parse_qos(declare_parameter<std::string>("imu_qos", "SENSOR_DATA"));

  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  const std::string base_frame =
      declare_parameter<std::string>("base_frame", "");
  declare_parameter<double>("tf_lookup_timeout_s", 5.0);

  std::string reference_frame =
      declare_parameter<std::string>("reference_frame", "");
  if (reference_frame.empty()) {
    const bool from_imu = rig_options.has_imu;
    const std::string &frame_topic =
        from_imu ? imu_topic : image_topics.front();
    reference_frame = from_imu ? wait_for_frame_id<sensor_msgs::msg::Imu>(
                                     *this, frame_topic, imu_qos)
                               : wait_for_frame_id<sensor_msgs::msg::Image>(
                                     *this, frame_topic, image_qos);
    RCLCPP_INFO(get_logger(),
                "reference_frame '%s', from the first message on '%s'",
                reference_frame.c_str(), frame_topic.c_str());
  }

  if (!base_frame.empty()) {
    rig_options.rig_from_reference = resolveRigFromReference(reference_frame);
    rig_frame_ = base_frame;
  } else {
    rig_frame_ = reference_frame;
  }

  BuiltRig built = build_rig(calib_, rig_options);
  rig_ = built.rig;
  imu_enabled_ = !rig_.imus.empty();

  RCLCPP_INFO(get_logger(),
              "Rig frame '%s' (anchored on %s): %zu camera(s), %zu imu(s), "
              "odometry mode %d, intra-process %s",
              rig_frame_.c_str(), rig_options.has_imu ? "imu" : "cam0",
              rig_.cameras.size(), rig_.imus.size(),
              static_cast<int>(built.config.odometry_mode),
              get_node_options().use_intra_process_comms()
                  ? "ACTIVE"
                  : "inactive (zero-copy disabled)");

  enable_slam_ = declare_parameter<bool>("enable_slam", true);
  const bool debug_exports = declare_parameter<bool>("debug_exports", false);
  exports_enabled_ = enable_slam_ || debug_exports;
  built.config.enable_observations_export = exports_enabled_;
  built.config.enable_landmarks_export = exports_enabled_;

  odometry_ = std::make_unique<cuvslam::Odometry>(rig_, built.config);

  if (enable_slam_) {
    cuvslam::Slam::Config slam_config = cuvslam::Slam::GetDefaultConfig();
    slam_config.max_map_size = static_cast<uint32_t>(
        declare_parameter<int64_t>("slam_max_map_size", 300));
    slam_config.throttling_time_ms = static_cast<uint32_t>(
        declare_parameter<int64_t>("slam_throttling_time_ms", 500));
    slam_ = std::make_unique<cuvslam::Slam>(
        rig_, odometry_->GetPrimaryCameras(), slam_config);
  }

  constexpr double kCuvslamMaxSpreadMs = 1.0;
  double sync_threshold_ms =
      declare_parameter<double>("sync_threshold_ms", 1.0);
  if (sync_threshold_ms > kCuvslamMaxSpreadMs) {
    RCLCPP_WARN(get_logger(),
                "sync_threshold_ms=%.3f exceeds cuVSLAM's 1 ms frame-sync "
                "limit; clamping to 1.0. "
                "Wider spreads would be rejected by Track().",
                sync_threshold_ms);
    sync_threshold_ms = kCuvslamMaxSpreadMs;
  }

  const int64_t min_num_images =
      declare_parameter<int64_t>("min_num_images", 0);
  const int64_t image_buffer_size =
      declare_parameter<int64_t>("image_buffer_size", 10);
  sync_ = std::make_unique<ImageSync>(
      num_cameras_, static_cast<int64_t>(sync_threshold_ms * 1e6),
      min_num_images > 0 ? static_cast<size_t>(min_num_images) : num_cameras_,
      static_cast<size_t>(std::max<int64_t>(1, image_buffer_size)));
  sync_->registerCallback(
      [this](int64_t stamp_ns, const std::vector<ImageSync::ImageMatch> &set) {
        callbackSynced(stamp_ns, set);
      });

  imu_queue_ = std::make_unique<ImuQueue>(static_cast<size_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("imu_buffer_size", 100))));

  cam_time_offset_ns_ = declare_parameter<bool>("apply_cam_time_offset", true)
                            ? calib_.cam_time_offset_ns
                            : 0;

  image_subs_.reserve(num_cameras_);
  for (size_t i = 0; i < num_cameras_; ++i) {
    image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
        image_topics[i], image_qos,
        [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          callbackImage(i, std::move(msg));
        }));
    RCLCPP_INFO(get_logger(), "camera %zu <- %s", i, image_topics[i].c_str());
  }

  if (imu_enabled_) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, imu_qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          callbackImu(std::move(msg));
        });
    RCLCPP_INFO(get_logger(), "imu <- %s", imu_topic.c_str());
  }

  // publishers
  publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", true);
  publish_map_tf_ = declare_parameter<bool>("publish_map_tf", true);

  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/pose", 10);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
  if (enable_slam_) {
    slam_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("~/slam_pose", 10);
  }

  const double path_publish_rate_hz =
      declare_parameter<double>("path_publish_rate_hz", 5.0);
  path_publish_period_ns_ =
      path_publish_rate_hz > 0.0
          ? static_cast<int64_t>(1e9 / path_publish_rate_hz)
          : 0;

  path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1));
  if (enable_slam_) {
    slam_path_pub_ =
        create_publisher<nav_msgs::msg::Path>("~/slam_path", rclcpp::QoS(1));
  }

  if (publish_odom_tf_ || (enable_slam_ && publish_map_tf_)) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // Debug views, only advertised if enabled.
  if (exports_enabled_) {
    observation_pubs_.reserve(num_cameras_);
    for (size_t i = 0; i < num_cameras_; ++i) {
      observation_pubs_.push_back(
          create_publisher<foxglove_msgs::msg::ImageAnnotations>(
              "~/cam" + std::to_string(i) + "/observations", 10));
    }
    landmark_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("~/landmarks", 10);
  }
}

Tracker::~Tracker() = default;

RigOptions Tracker::readRigOptions() {
  RigOptions options;
  options.on_warning = [this](const std::string &warning) {
    RCLCPP_WARN(get_logger(), "%s", warning.c_str());
  };

  const auto reference =
      declare_parameter<std::string>("rig_reference", "auto");
  if (reference == "auto") {
    options.has_imu = calib_.imu_update_rate > 0.0;
  } else if (reference == "imu") {
    options.has_imu = true;
  } else if (reference == "cam0") {
    options.has_imu = false;
  } else {
    throw std::runtime_error(
        "rig_reference must be 'auto', 'imu' or 'cam0', got '" + reference +
        "'");
  }

  const auto mode = declare_parameter<std::string>("odometry_mode", "auto");
  if (mode == "auto") {
    options.mode = std::nullopt;
  } else if (mode == "multicamera") {
    options.mode = cuvslam::Odometry::OdometryMode::Multicamera;
  } else if (mode == "inertial") {
    options.mode = cuvslam::Odometry::OdometryMode::Inertial;
  } else if (mode == "mono") {
    options.mode = cuvslam::Odometry::OdometryMode::Mono;
  } else if (mode == "multisensor") {
    options.mode = cuvslam::Odometry::OdometryMode::Multisensor;
  } else {
    throw std::runtime_error(
        "odometry_mode must be auto|multicamera|inertial|mono|multisensor, got "
        "'" +
        mode + "'");
  }

  const auto multicam =
      declare_parameter<std::string>("multicam_mode", "precision");
  if (multicam == "precision") {
    options.multicam_mode = cuvslam::Odometry::MulticameraMode::Precision;
  } else if (multicam == "performance") {
    options.multicam_mode = cuvslam::Odometry::MulticameraMode::Performance;
  } else if (multicam == "moderate") {
    options.multicam_mode = cuvslam::Odometry::MulticameraMode::Moderate;
  } else {
    throw std::runtime_error(
        "multicam_mode must be precision|performance|moderate, got '" +
        multicam + "'");
  }

  options.use_imu = declare_parameter<bool>("use_imu", true);
  options.imu_noise_scale = declare_parameter<double>("imu_noise_scale", 1.0);
  options.image_jitter_threshold_ms =
      declare_parameter<double>("image_jitter_threshold_ms", 34.0);

  const auto optional_positive = [this](const std::string &name) {
    const double value = declare_parameter<double>(name, -1.0);
    return value > 0.0 ? std::optional<double>(value) : std::nullopt;
  };
  options.gyroscope_noise_density =
      optional_positive("gyroscope_noise_density");
  options.gyroscope_random_walk = optional_positive("gyroscope_random_walk");
  options.accelerometer_noise_density =
      optional_positive("accelerometer_noise_density");
  options.accelerometer_random_walk =
      optional_positive("accelerometer_random_walk");

  return options;
}

Sophus::SE3d
Tracker::resolveRigFromReference(const std::string &reference_frame) {
  const std::string base_frame = get_parameter("base_frame").as_string();
  const double timeout = get_parameter("tf_lookup_timeout_s").as_double();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  try {
    const auto tf = tf_buffer_->lookupTransform(base_frame, reference_frame,
                                                tf2::TimePointZero,
                                                tf2::durationFromSec(timeout));
    const auto &t = tf.transform.translation;
    const auto &r = tf.transform.rotation;
    const Sophus::SE3d result(
        Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized(),
        Eigen::Vector3d(t.x, t.y, t.z));
    RCLCPP_INFO(get_logger(),
                "Re-basing rig onto '%s': %s <- %s = [t=(%.4f, %.4f, %.4f) "
                "q=(%.4f, %.4f, %.4f, %.4f)]",
                base_frame.c_str(), base_frame.c_str(), reference_frame.c_str(),
                t.x, t.y, t.z, r.x, r.y, r.z, r.w);
    return result;
  } catch (const tf2::TransformException &e) {
    RCLCPP_WARN(get_logger(),
                "Could not look up '%s' <- '%s' within %.1f s (%s). ASSUMING "
                "IDENTITY: the rig stays on "
                "'%s'. If those frames are not physically coincident, the "
                "published poses are offset by the "
                "missing transform.",
                base_frame.c_str(), reference_frame.c_str(), timeout, e.what(),
                reference_frame.c_str());
    return Sophus::SE3d{};
  }
}

void Tracker::callbackImage(size_t camera_index,
                            sensor_msgs::msg::Image::ConstSharedPtr msg) {
  const auto &expected = rig_.cameras[camera_index].size;
  if (static_cast<int32_t>(msg->width) != expected[0] ||
      static_cast<int32_t>(msg->height) != expected[1]) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "camera %zu: image is %ux%u but the calibration says "
                          "%dx%d; dropping frames",
                          camera_index, msg->width, msg->height, expected[0],
                          expected[1]);
    return;
  }

  const char *const target =
      sensor_msgs::image_encodings::isColor(msg->encoding)
          ? sensor_msgs::image_encodings::RGB8
          : sensor_msgs::image_encodings::MONO8;
  cv_bridge::CvImageConstPtr shared;
  try {
    shared = cv_bridge::toCvShare(msg, target);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "camera %zu: cannot read '%s' as %s (%s); dropping "
                          "frames",
                          camera_index, msg->encoding.c_str(), target,
                          e.what());
    return;
  }

  if (shared->image.data != msg->data.data() || !shared->image.isContinuous()) {
    RCLCPP_WARN_ONCE(
        get_logger(),
        "camera %zu: %s with %u byte rows cannot be tracked in place; "
        "converting to %s on every frame.",
        camera_index, msg->encoding.c_str(), msg->step, target);
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                          "camera %zu: converting %s -> %s", camera_index,
                          msg->encoding.c_str(), target);
    if (!shared->image.isContinuous()) {
      // Convert image to packed format.
      shared = std::make_shared<const cv_bridge::CvImage>(
          shared->header, shared->encoding, shared->image.clone());
    }
  }

  const int64_t stamp_ns =
      rclcpp::Time(msg->header.stamp).nanoseconds() - cam_time_offset_ns_;
  sync_->addImage(camera_index, stamp_ns, std::move(shared));
}

void Tracker::callbackImu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  cuvslam::ImuMeasurement measurement;
  measurement.timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  measurement.linear_accelerations = {
      static_cast<float>(msg->linear_acceleration.x),
      static_cast<float>(msg->linear_acceleration.y),
      static_cast<float>(msg->linear_acceleration.z)};
  measurement.angular_velocities = {
      static_cast<float>(msg->angular_velocity.x),
      static_cast<float>(msg->angular_velocity.y),
      static_cast<float>(msg->angular_velocity.z)};
  imu_queue_->push(measurement);
}

void Tracker::callbackSynced(int64_t stamp_ns,
                             const std::vector<ImageSync::ImageMatch> &set) {
  if (imu_enabled_) {
    for (const auto &measurement : imu_queue_->popUntil(stamp_ns)) {
      odometry_->RegisterImuMeasurement(0, measurement);
    }
  }

  // zero-copy handoff, cuVSLAM reads straight out of the frame buffers while
  // set holds them alive.
  std::vector<cuvslam::Image> images;
  images.reserve(set.size());
  for (const auto &match : set) {
    const auto &frame = *match.image;
    cuvslam::Image image;
    image.pixels = frame.image.data;
    image.width = static_cast<int32_t>(frame.image.cols);
    image.height = static_cast<int32_t>(frame.image.rows);
    image.pitch =
        static_cast<int32_t>(frame.image.step); // ignored for host images
    image.encoding = frame.encoding == "mono8"
                         ? cuvslam::ImageData::Encoding::MONO
                         : cuvslam::ImageData::Encoding::RGB;
    image.data_type = cuvslam::ImageData::DataType::UINT8;
    image.is_gpu_mem = false;
    image.timestamp_ns = match.stamp_ns;
    image.camera_index = static_cast<uint32_t>(match.camera_index);
    images.push_back(image);
  }

  const auto started = std::chrono::steady_clock::now();
  cuvslam::PoseEstimate estimate;
  try {
    estimate = odometry_->Track(images);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "cuVSLAM Track() failed: %s", e.what());
    return;
  }
  const double track_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - started)
                              .count();

  ++frame_count_;
  if (!estimate.world_from_rig.has_value()) {
    ++tracking_failures_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "tracking lost (%lu of %lu frames)",
                         static_cast<unsigned long>(tracking_failures_),
                         static_cast<unsigned long>(frame_count_));
    publishDiagnostics(estimate.timestamp_ns, false, track_ms);
    return;
  }

  publishPose(estimate.timestamp_ns, *estimate.world_from_rig);

  if (exports_enabled_) {
    try {
      publishObservations(estimate.timestamp_ns);
      publishLandmarks(estimate.timestamp_ns);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "cuVSLAM debug view failed: %s", e.what());
    }
  }

  if (slam_) {
    try {
      cuvslam::Odometry::State state;
      odometry_->GetState(state);
      slam_->Track(state);
      publishSlam(estimate.timestamp_ns,
                  cuvslam_to_sophus(estimate.world_from_rig->pose));
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "cuVSLAM SLAM step failed: %s. Odometry continues; "
                            "map -> odom will be stale.",
                            e.what());
    }
  }

  publishDiagnostics(estimate.timestamp_ns, true, track_ms);
}

void Tracker::publishPose(int64_t stamp_ns,
                          const cuvslam::PoseWithCovariance &world_from_rig) {
  const rclcpp::Time stamp(stamp_ns);
  const Sophus::SE3d odom_from_rig = cuvslam_to_sophus(world_from_rig.pose);

  if (publish_odom_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = rig_frame_;
    tf.transform = to_transform_msgs(odom_from_rig);
    tf_broadcaster_->sendTransform(tf);
  }

  if (has_subscribers(pose_pub_)) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.pose.pose = to_pose_msg(odom_from_rig);
    std::copy(world_from_rig.covariance_xyz_rpy.begin(),
              world_from_rig.covariance_xyz_rpy.end(),
              msg.pose.covariance.begin());
    pose_pub_->publish(msg);
  }

  if (has_subscribers(odometry_pub_)) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = rig_frame_;
    msg.pose.pose = to_pose_msg(odom_from_rig);
    std::copy(world_from_rig.covariance_xyz_rpy.begin(),
              world_from_rig.covariance_xyz_rpy.end(),
              msg.pose.covariance.begin());

    // Body-frame twist from a two-pose finite difference, which is what
    // child_frame_id implies.
    if (has_previous_ && stamp_ns > previous_stamp_ns_) {
      const double dt =
          static_cast<double>(stamp_ns - previous_stamp_ns_) * 1e-9;
      const Sophus::SE3d delta =
          previous_odom_from_rig_.inverse() * odom_from_rig;
      const Eigen::Vector3d linear = delta.translation() / dt;
      const Eigen::Vector3d angular = delta.so3().log() / dt;
      msg.twist.twist.linear.x = linear.x();
      msg.twist.twist.linear.y = linear.y();
      msg.twist.twist.linear.z = linear.z();
      msg.twist.twist.angular.x = angular.x();
      msg.twist.twist.angular.y = angular.y();
      msg.twist.twist.angular.z = angular.z();
    }
    odometry_pub_->publish(msg);
  }

  auto &new_path_entry = path_.emplace_back();
  new_path_entry.header.stamp = stamp;
  new_path_entry.header.frame_id = odom_frame_;
  new_path_entry.pose = to_pose_msg(odom_from_rig);

  if (has_subscribers(path_pub_) &&
      is_due(last_path_publish_ns_, stamp_ns, path_publish_period_ns_)) {
    nav_msgs::msg::Path msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.poses.assign(path_.begin(), path_.end());
    path_pub_->publish(msg);
  }

  previous_odom_from_rig_ = odom_from_rig;
  previous_stamp_ns_ = stamp_ns;
  has_previous_ = true;
}

void Tracker::publishSlam(int64_t stamp_ns, const Sophus::SE3d &odom_from_rig) {
  const rclcpp::Time stamp(stamp_ns);
  const Sophus::SE3d map_from_rig = cuvslam_to_sophus(slam_->GetPose());

  if (has_subscribers(slam_pose_pub_)) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose = to_pose_msg(map_from_rig);
    slam_pose_pub_->publish(msg);
  }

  if (has_subscribers(slam_path_pub_) &&
      is_due(last_slam_path_publish_ns_, stamp_ns, path_publish_period_ns_)) {
    std::vector<cuvslam::PoseStamped> poses;
    slam_->GetAllSlamPoses(poses);

    nav_msgs::msg::Path msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.poses.reserve(poses.size());
    for (const auto &pose : poses) {
      auto &entry = msg.poses.emplace_back();
      entry.header.stamp = rclcpp::Time(pose.timestamp_ns);
      entry.header.frame_id = map_frame_;
      entry.pose = to_pose_msg(cuvslam_to_sophus(pose.pose));
    }
    slam_path_pub_->publish(msg);
  }

  if (publish_map_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform = to_transform_msgs(map_from_rig * odom_from_rig.inverse());
    tf_broadcaster_->sendTransform(tf);
  }
}

void Tracker::publishObservations(int64_t stamp_ns) {
  const rclcpp::Time stamp(stamp_ns);

  for (size_t i = 0; i < observation_pubs_.size(); ++i) {
    if (!has_subscribers(observation_pubs_[i])) {
      continue;
    }

    foxglove_msgs::msg::ImageAnnotations msg;
    msg.timestamp = stamp;
    auto &annotation = msg.points.emplace_back();
    annotation.type = foxglove_msgs::msg::PointsAnnotation::POINTS;
    annotation.thickness = 3.0;

    // cuVSLAM already projected through the real camera model, so u/v go
    // straight across.
    for (const auto &observation :
         odometry_->GetLastObservations(static_cast<uint32_t>(i))) {
      foxglove_msgs::msg::Point2 point;
      point.x = observation.u;
      point.y = observation.v;
      annotation.points.push_back(point);
      annotation.outline_colors.push_back(color_from_id(observation.id));
    }

    observation_pubs_[i]->publish(std::move(msg));
  }
}

void Tracker::publishLandmarks(int64_t stamp_ns) {
  if (!has_subscribers(landmark_pub_)) {
    return;
  }

  const std::vector<cuvslam::Landmark> landmarks =
      odometry_->GetLastLandmarks();

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = rclcpp::Time(stamp_ns);
  cloud.header.frame_id = rig_frame_;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(landmarks.size());

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const auto &landmark : landmarks) {
    *x = landmark.coords[0];
    *y = landmark.coords[1];
    *z = landmark.coords[2];
    ++x;
    ++y;
    ++z;
  }

  landmark_pub_->publish(std::move(cloud));
}

void Tracker::publishDiagnostics(int64_t stamp_ns, bool tracking,
                                 double track_ms) {
  if (!has_subscribers(status_pub_)) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(get_name()) + ": tracking";
  status.hardware_id = rig_frame_;
  status.level = tracking ? diagnostic_msgs::msg::DiagnosticStatus::OK
                          : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = tracking ? "tracking" : "tracking lost";

  const auto add = [&status](const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };
  add("frames", std::to_string(frame_count_));
  add("tracking_failures", std::to_string(tracking_failures_));
  add("track_ms", std::to_string(track_ms));
  add("imu_queued", std::to_string(imu_queue_->size()));
  add("imu_dropped", std::to_string(imu_queue_->dropped()));

  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = rclcpp::Time(stamp_ns);
  msg.status.push_back(std::move(status));

  status_pub_->publish(msg);
}

} // namespace cairn

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cairn::Tracker)
