#include "virtcam/spherical.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "filesystem"
#include "opencv2/core/eigen.hpp"
#include "synchronizer.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace virtcam {

SphericalCamera::SphericalCamera(const rclcpp::NodeOptions &options)
    : Node("virtual_camera", options) {
  this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener =
      std::make_unique<tf2_ros::TransformListener>(*this->tf_buffer);

  this->pre_set_param_callback_handle = this->add_pre_set_parameters_callback(
      std::bind(&SphericalCamera::pre_set_param_callback, this,
                std::placeholders::_1));
  this->on_set_param_callback_handle = this->add_on_set_parameters_callback(
      std::bind(&SphericalCamera::on_set_param_callback, this,
                std::placeholders::_1));
  this->post_set_param_callback_handle = this->add_post_set_parameters_callback(
      std::bind(&SphericalCamera::post_set_param_callback, this,
                std::placeholders::_1));

  this->initialized = false;
  this->declare_parameter("input.calibration", "");
  this->declare_parameter("input.prefix", "cam");
  this->declare_parameter("input.depth", 1.0);
  this->declare_parameter("input.vignette_threshold", 0.5);
  this->declare_parameter("input.use_mask_as_vignette", false);
  this->declare_parameter("input.frame_id", "imu");

  this->declare_parameter("output.frame_id", std::string(this->get_name()) + "_optical");
  this->declare_parameter("output.res_x", 0);
  this->declare_parameter("output.res_y", 720);
  this->declare_parameter("output.fov_x", M_PI * 2.0);
  this->declare_parameter("output.fov_y", M_PI);

  this->build_intrinsics();
  this->build_subscriptions();
  this->initialized = true;

  this->pub = this->create_publisher<sensor_msgs::msg::Image>(
      "~/image_raw", rclcpp::SensorDataQoS());
}

void SphericalCamera::image_callback(
    const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> &msgs) {
  std::vector<cv::Mat> imgs;
  std::vector<float> expos;
  imgs.reserve(msgs.size());
  expos.reserve(msgs.size());
  for (const auto &msg : msgs) {
    auto cvb = cv_bridge::toCvShare(msg);
    imgs.push_back(cvb->image);
    try {
      expos.emplace_back(std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::nanoseconds(std::stoul(msg->header.frame_id))).count());
    } catch (std::exception const& ex) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Unable to read exposure time from frame_id, with error \"%s\", no exposure compensation will be applied.", ex.what());
      expos.emplace_back(1.0f);
    }
  }

  const auto frame_base = this->get_parameter("output.frame_id").as_string();
  const auto frame_imu = this->get_parameter("input.frame_id").as_string();
  try {
    const auto tf = this->tf_buffer->lookupTransform(frame_imu, frame_base, tf2::TimePointZero);
    auto extr = Sophus::SE3f(tf2::transformToEigen(tf).matrix().cast<float>());
    if (!extr.matrix3x4().isApprox(this->extr_last.matrix3x4())) {
      cv::Affine3f::Mat4 extr_mat;
      cv::eigen2cv(extr.matrix(), extr_mat);
      this->stitcher.transform(extr_mat);
      this->extr_last = extr;
    }
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not get transform between %s and %s: %s", frame_base.c_str(), frame_imu.c_str(), ex.what());
  }

  auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
  out_msg->header.frame_id = this->get_parameter("output.frame_id").as_string();
  out_msg->header.stamp = msgs.front()->header.stamp;
  out_msg->encoding = msgs.front()->encoding;
  out_msg->height = this->stitcher.height();
  out_msg->width = this->stitcher.width();
  out_msg->step = out_msg->width * imgs.front().elemSize();
  out_msg->data.resize(out_msg->step * out_msg->height);
  cv::Mat out_cv(out_msg->height, out_msg->width, imgs.front().type(),
                 out_msg->data.data());
  this->stitcher.stitch(imgs, expos, out_cv);

  this->pub->publish(std::move(out_msg));
}

void SphericalCamera::pre_set_param_callback(
    std::vector<rclcpp::Parameter> &params) {
  bool rx = false, ry = false, fx = false, fy = false;
  for (auto &param : params) {
    rx = rx || param.get_name() == "output.res_x";
    ry = ry || param.get_name() == "output.res_y";
    fx = fx || param.get_name() == "output.fov_x";
    fy = fy || param.get_name() == "output.fov_y";
  }
  if (rx || ry || fx || fy) {
    if (!rx) {
      params.push_back(this->get_parameter("output.res_x"));
    }
    if (!ry) {
      params.push_back(this->get_parameter("output.res_y"));
    }
    if (!fx) {
      params.push_back(this->get_parameter("output.fov_x"));
    }
    if (!fy) {
      params.push_back(this->get_parameter("output.fov_y"));
    }
  }
}

rcl_interfaces::msg::SetParametersResult SphericalCamera::on_set_param_callback(
    const std::vector<rclcpp::Parameter> &params) {
  rcl_interfaces::msg::SetParametersResult result;
  uint8_t placeholder_count = 0;

  for (const auto &param : params) {
    if (param.get_name() == "input.calibration") {
      const auto path = param.as_string();
      if (path.size() == 0) {
        result.successful = false;
        result.reason = "Calibration file path cannot be empty";
        return result;
      }
      if (!std::filesystem::exists(path)) {
        result.successful = false;
        result.reason = "Calibration file does not exist";
        return result;
      }
    } else if (param.get_name() == "input.vignette_threshold") {
      const auto vt = param.as_double();
      if (vt < 0.0 || vt > 1.0) {
        result.successful = false;
        result.reason = "Vignette threshold must be in [0, 1] ";
        return result;
      }
    } else if (param.get_name() == "output.res_x") {
      const auto res_x = param.as_int();
      if (res_x <= 0) {
        placeholder_count++;
      }
    } else if (param.get_name() == "output.res_y") {
      const auto res_y = param.as_int();
      if (res_y <= 0) {
        placeholder_count++;
      }
    } else if (param.get_name() == "output.fov_x") {
      const auto fov_x = param.as_double();
      if (fov_x <= 0.0) {
        placeholder_count++;
      } else if (fov_x > 2.0 * M_PI) {
        result.successful = false;
        result.reason = "Horizontal FOV must be in (0, 2*pi] radians";
        return result;
      }
    } else if (param.get_name() == "output.fov_y") {
      const auto fov_y = param.as_double();
      if (fov_y <= 0.0) {
        placeholder_count++;
      } else if (fov_y > M_PI) {
        result.successful = false;
        result.reason = "Vertical FOV must be in (0, pi] radians";
        return result;
      }
    }
  }

  if (placeholder_count > 1) {
    result.successful = false;
    result.reason = "At most one of output.res_x, output.res_y, "
                    "output.fov_x, output.fov_y "
                    "can be set to a placeholder value (<=0)";
    return result;
  }

  result.successful = true;
  result.reason = "";
  return result;
}

void SphericalCamera::post_set_param_callback(
    const std::vector<rclcpp::Parameter> &params) {
  bool build_intr = false, build_subs = false;
  for (auto &param : params) {
    if (param.get_name() == "input.calibration") {
      this->stitcher.loadCalibration(param.as_string());
      build_subs = true;
    } else if (param.get_name() == "input.prefix" ||
               param.get_name() == "input.depth") {
      build_subs = true;
    } else if (param.get_name() == "input.vignette_threshold") {
      this->stitcher.vignetteThreshold(param.as_double());
    } else if (param.get_name() == "input.use_mask_as_vignette") {
      this->stitcher.useMaskAsVignette(param.as_bool());
    } else if (param.get_name() == "output.res_x" ||
               param.get_name() == "output.res_y" ||
               param.get_name() == "output.fov_x" ||
               param.get_name() == "output.fov_y") {
      build_intr = true;
    }
  }

  if (this->initialized) {
    if (build_intr) {
      this->build_intrinsics();
    }
    if (build_subs) {
      this->build_subscriptions();
    }
  }
}

void SphericalCamera::build_intrinsics() {
  auto res_x = this->get_parameter("output.res_x").as_int();
  auto res_y = this->get_parameter("output.res_y").as_int();
  auto fov_x = this->get_parameter("output.fov_x").as_double();
  auto fov_y = this->get_parameter("output.fov_y").as_double();

  double aspect;
  if (res_x <= 0 || res_y <= 0) {
    aspect = fov_x / fov_y;
  } else if (fov_x <= 0.0 || fov_y <= 0.0) {
    aspect = static_cast<double>(res_x) / static_cast<double>(res_y);
  }
  if (res_x <= 0) {
    res_x = static_cast<int>(std::round(res_y * aspect));
  } else if (res_y <= 0) {
    res_y = static_cast<int>(std::round(res_x / aspect));
  }
  if (fov_x <= 0.0) {
    fov_x = fov_y * aspect;
  } else if (fov_y <= 0.0) {
    fov_y = fov_x / aspect;
  }
  this->stitcher.resolution(res_x, res_y);
  this->stitcher.fov(fov_x, fov_y);
}

void SphericalCamera::build_subscriptions() {
  std::vector<std::string> topics = this->stitcher.calibration().cam_names;
  if (topics.empty()) {
    const auto prefix = this->get_parameter("input.prefix").as_string();
    topics.resize(this->stitcher.calibration().num_cams());
    for (std::size_t i = 0; i < topics.size(); ++i) {
      topics[i] = prefix + std::to_string(i) + "/image_raw";
    }
  }
  this->sub_sync = std::make_unique<VariableSynchronizer>(
      this, topics,
      std::bind(&SphericalCamera::image_callback, this, std::placeholders::_1),
      rclcpp::SensorDataQoS());

  const auto depth = this->get_parameter("input.depth").as_double();
  this->sub_depth.reset();
  if (depth > 0.0) {
    this->stitcher.setDepth(depth);
  } else {
    this->sub_depth = this->create_subscription<sensor_msgs::msg::Image>(
        "depth/image_raw", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
          const auto cvb = cv_bridge::toCvShare(msg);
          this->stitcher.setDepth(cvb->image);
        });
  }
}

} // namespace virtcam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(virtcam::SphericalCamera)
