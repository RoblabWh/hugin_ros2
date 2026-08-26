#include "virtcam/perspective.hpp"
#include "basalt/serialization/headers_serialization.h"
#include "cv_bridge/cv_bridge.hpp"
#include "filesystem"
#include "tf2_eigen/tf2_eigen.hpp"

namespace virtcam {

PerspectiveCamera::PerspectiveCamera(const rclcpp::NodeOptions &options)
    : Node("virtual_camera", options) {
  this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener =
      std::make_unique<tf2_ros::TransformListener>(*this->tf_buffer);
  this->tfs_broadcaster =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  this->pre_set_param_callback_handle =
      this->add_pre_set_parameters_callback(std::bind(
          &PerspectiveCamera::pre_set_param_callback, this, std::placeholders::_1));
  this->on_set_param_callback_handle =
      this->add_on_set_parameters_callback(std::bind(
          &PerspectiveCamera::on_set_param_callback, this, std::placeholders::_1));
  this->post_set_param_callback_handle = this->add_post_set_parameters_callback(
      std::bind(&PerspectiveCamera::post_set_param_callback, this,
                std::placeholders::_1));

  this->initialized = false;
  this->declare_parameter("input.calibration", "");
  this->declare_parameter("input.index", 0);
  this->declare_parameter("input.depth", 1.0);
  this->declare_parameter("input.frame_id", "imu");

  this->declare_parameter("output.frame_id", std::string(this->get_name()) + "_optical");
  this->declare_parameter("output.type", "pinhole");
  this->declare_parameter("output.res_x", 1280);
  this->declare_parameter("output.res_y", 720);
  this->declare_parameter("output.fov_x", M_PI_2);
  this->declare_parameter("output.fov_y", 0.0);
  this->declare_parameter("output.intrinsics", std::vector<double>{});

  this->declare_parameter("output.attach_to", std::vector<int64_t>{});
  this->declare_parameter("output.attach_offset", 0.5);

  if (!this->build_intrinsics()) {
    throw std::runtime_error("Invalid output camera configuration");
  }
  this->build_extrinsics();
  this->build_map();
  this->build_subscription();
  this->initialized = true;

  this->pub = this->create_publisher<sensor_msgs::msg::Image>(
      "~/image_raw", rclcpp::SensorDataQoS());
}

void PerspectiveCamera::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &in_msg) {
  auto in_cvb = cv_bridge::toCvShare(in_msg);

  static bool tf_ok = true;
  if (this->get_parameter("output.attach_to").as_integer_array().empty()) {
    const auto frame_base = this->get_parameter("output.frame_id").as_string();
    const auto frame_imu = this->get_parameter("input.frame_id").as_string();
    try {
      const auto tf = this->tf_buffer->lookupTransform(frame_imu, frame_base, tf2::TimePointZero);
      const auto extr = Sophus::SE3f(tf2::transformToEigen(tf).matrix().cast<float>());
      if (!extr.matrix3x4().isApprox(this->out_extr.matrix3x4())) {
        if (!tf_ok) {
          RCLCPP_INFO(this->get_logger(), "Extrinsics updated with transform between \"%s\" and \"%s\"", frame_base.c_str(), frame_imu.c_str());
          tf_ok = true;
        }
        this->out_extr = extr;
        this->build_map();
      }
    } catch (tf2::TransformException &ex) {
      if (tf_ok) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform between \"%s\" and \"%s\": %s", frame_base.c_str(), frame_imu.c_str(), ex.what());
        tf_ok = false;
      }
    }
  }

  auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
  out_msg->header.frame_id = this->get_parameter("output.frame_id").as_string();
  out_msg->header.stamp = in_msg->header.stamp;
  out_msg->encoding = in_msg->encoding;
  out_msg->height = this->map.rows;
  out_msg->width = this->map.cols;
  out_msg->step = this->map.cols * in_cvb->image.elemSize();
  out_msg->data.resize(out_msg->step * out_msg->height);
  cv::Mat out_cv(out_msg->height, out_msg->width, in_cvb->image.type(),
                 out_msg->data.data());

  cv::remap(in_cvb->image, out_cv, this->map, cv::noArray(), cv::INTER_LINEAR);

  this->pub->publish(std::move(out_msg));
}

void PerspectiveCamera::pre_set_param_callback(
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

rcl_interfaces::msg::SetParametersResult PerspectiveCamera::on_set_param_callback(
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
    } else if (param.get_name() == "input.index") {
      const auto index = param.as_int();
      const auto num_cams = this->in_calib.num_cams();
      if (index < 0 || static_cast<size_t>(index) >= num_cams) {
        result.successful = false;
        result.reason =
            "Index must be in [0, " + std::to_string(num_cams - 1) + "]";
        return result;
      }
    } else if (param.get_name() == "input.depth") {
      const auto depth = param.as_double();
      if (depth <= 0.0) {
        result.successful = false;
        result.reason = "Depth must be positive";
        return result;
      }
    } else if (param.get_name() == "output.type") {
      const auto type = param.as_string();
      if (!this->out_intr.isValidType(type)) {
        result.successful = false;
        result.reason = "Camera type not recognized";
        return result;
      }
    } else if (param.get_name() == "output.attach_to") {
      const auto attachments = param.as_integer_array();
      if (attachments.size() > 2) {
        result.successful = false;
        result.reason = "Only attaching to up to two cameras is supported";
        return result;
      }
      const auto num_cams = this->in_calib.num_cams();
      for (const auto &index : attachments) {
        if (index < 0 || static_cast<size_t>(index) >= num_cams) {
          result.successful = false;
          result.reason = "Attachment indices must be in [0, " +
                          std::to_string(num_cams - 1) + "]";
          return result;
        }
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
      } else if (fov_y > 2.0 * M_PI) {
        result.successful = false;
        result.reason = "Vertical FOV must be in (0, 2*pi] radians";
        return result;
      }
    }
  }

  if (placeholder_count > 1) {
    result.successful = false;
    result.reason =
        "At most one of output.res_x, output.res_y, output.fov_x, output.fov_y "
        "can be set to a placeholder value (<=0)";
    return result;
  }

  result.successful = true;
  result.reason = "";
  return result;
}

void PerspectiveCamera::post_set_param_callback(
    const std::vector<rclcpp::Parameter> &params) {
  bool build_intr = false, build_extr = false, build_map = false, build_sub = false;

  for (auto &param : params) {
    if (param.get_name() == "input.calibration") {
      const auto path = param.as_string();
      std::ifstream file(path);
      cereal::JSONInputArchive archive(file);
      archive(this->in_calib);
      build_extr = true;
      build_sub = true;
    } else if (param.get_name() == "input.index") {
      build_extr = true;
      build_sub = true;
    } else if (param.get_name() == "input.depth") {
      build_map = true;
    } else if (param.get_name() == "input.frame_id" ||
               param.get_name() == "output.frame_id" ||
               param.get_name() == "output.attach_to" ||
               param.get_name() == "output.attach_offset") {
      build_extr = true;
    } else if (param.get_name() == "output.type" ||
               param.get_name() == "output.res_x" ||
               param.get_name() == "output.res_y" ||
               param.get_name() == "output.fov_x" ||
               param.get_name() == "output.fov_y" ||
               param.get_name() == "output.intrinsics") {
      build_intr = true;
    }
  }

  if (this->initialized) {
    if (build_intr) {
      this->build_intrinsics();
      build_map = true;
    }
    if (build_extr) {
      this->build_extrinsics();
      build_map = true;
    }
    if (build_map) {
      this->build_map();
    }
    if (build_sub) {
      this->build_subscription();
    }
  }
}

bool PerspectiveCamera::build_intrinsics() {
  const auto type = this->get_parameter("output.type").as_string();
  auto res_x = this->get_parameter("output.res_x").as_int();
  auto res_y = this->get_parameter("output.res_y").as_int();
  const auto fov_x = this->get_parameter("output.fov_x").as_double();
  const auto fov_y = this->get_parameter("output.fov_y").as_double();
  const auto intr_extra =
      this->get_parameter("output.intrinsics").as_double_array();

  const auto create_camera = [&](float fx, float fy, float cx, float cy) {
    auto cam = this->out_intr.fromString(type);
    cam.setFromInit(Eigen::Vector4f(fx, fy, cx, cy));
    const auto intr_num = cam.getN();
    Eigen::VectorXf intr_inc = Eigen::VectorXf::Zero(intr_num);
    for (std::size_t i = 0;
         i < std::min<std::size_t>(intr_extra.size(), intr_num - 4); ++i) {
      intr_inc[i + 4] = intr_extra[i] - cam.getParam()[i + 4];
    }
    cam.applyInc(intr_inc);
    return cam;
  };

  // Probe the projection model with unit focal length and zero centre, so
  // projecting a bearing yields the image radius in normalized units.
  auto probe = create_camera(1.0f, 1.0f, 0.0f, 0.0f);

  bool good = true;
  const auto radius_at = [&](double fov) {
    const auto half_fov = fov * 0.5;
    Eigen::Vector2f p2d = Eigen::Vector2f::Zero();
    good &= probe.project(
        Eigen::Vector3f(std::sin(half_fov), 0.0f, std::cos(half_fov)), p2d);
    return static_cast<double>(p2d.x());
  };

  double aspect = 1.0;
  if (res_x <= 0 || res_y <= 0) {
    aspect = radius_at(fov_x) / radius_at(fov_y);
  } else {
    aspect = static_cast<double>(res_x) / static_cast<double>(res_y);
  }
  good &= std::isfinite(aspect) && aspect > 0.0;

  const auto rx = fov_x > 0.0 ? radius_at(fov_x) : radius_at(fov_y) * aspect;
  const auto ry = fov_y > 0.0 ? radius_at(fov_y) : radius_at(fov_x) / aspect;

  Eigen::Vector3f bearing;
  good &= probe.unproject(Eigen::Vector2f(static_cast<float>(rx), 0.0f), bearing);
  good &= probe.unproject(Eigen::Vector2f(0.0f, static_cast<float>(ry)), bearing);

  if (!good) {
    RCLCPP_ERROR(this->get_logger(),
                 "Cannot derive intrinsics from resolution (%ld, %ld) and "
                 "field of view (%f, %f) with the %s model",
                 res_x, res_y, fov_x, fov_y, probe.getName().c_str());
    return false;
  }

  if (res_x <= 0) {
    res_x = static_cast<int>(std::round(res_y * aspect));
  } else if (res_y <= 0) {
    res_y = static_cast<int>(std::round(res_x / aspect));
  }

  const auto cx = res_x * 0.5;
  const auto cy = res_y * 0.5;

  this->out_intr = create_camera(cx / rx, cy / ry, cx, cy);
  this->map.create(res_y, res_x, CV_32FC2);
  return true;
}

void PerspectiveCamera::build_extrinsics() {
  const auto attach_to = this->get_parameter("output.attach_to").as_integer_array();

  if (attach_to.size() == 1) {
    this->out_extr = this->in_calib.T_i_c[attach_to[0]];
  } else if (attach_to.size() == 2) {
    const auto attach_offset = this->get_parameter("output.attach_offset").as_double();
    const auto &t1 = this->in_calib.T_i_c[attach_to[0]];
    const auto &t2 = this->in_calib.T_i_c[attach_to[1]];

    const auto ti = t1.translation() + (t2.translation() - t1.translation()) * attach_offset;
    const auto ri = t1.so3() * t1.so3().exp((t1.so3().inverse() * t2.so3()).log() * 0.5f);

    this->out_extr = Sophus::SE3f(ri, ti);
  } else {
    const auto index = this->get_parameter("input.index").as_int();
    this->out_extr = this->in_calib.T_i_c[index];
  }

  if (attach_to.size() >= 1) {
    auto tf = tf2::eigenToTransform(Eigen::Affine3d(this->out_extr.matrix().cast<double>()));
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = this->get_parameter("input.frame_id").as_string();
    tf.child_frame_id = this->get_parameter("output.frame_id").as_string();
    this->tfs_broadcaster->sendTransform(tf);
  }
}

void PerspectiveCamera::build_map() {
  const auto in_idx = this->get_parameter("input.index").as_int();
  const auto in_intr = this->in_calib.intrinsics[in_idx];
  const auto in_extr = this->in_calib.T_i_c[in_idx];
  const auto in_depth = this->get_parameter("input.depth").as_double();

  auto map = this->map.getMat(cv::ACCESS_WRITE);
  std::visit(
      [&](const auto &in_cam) {
        std::visit(
            [&](const auto &out_cam) {
              const auto T_in_out = in_extr.inverse() * this->out_extr;
              map.forEach<cv::Vec2f>([&](auto &mapping, auto pos) -> void {
                Eigen::Vector3f p3d;
                auto p2d = Eigen::Map<Eigen::Vector2f>(mapping.val);

                bool good = out_cam.unproject(Eigen::Vector2f{pos[1] + 0.5f, pos[0] + 0.5f}, p3d);
                if (good) {
                  p3d = T_in_out * (p3d * static_cast<float>(in_depth));
                  good = in_cam.project(p3d, p2d);
                }
                if (!good) {
                  p2d.setConstant(-1.0f);
                }
              });
            },
            out_intr.variant);
      },
      in_intr.variant);
}

void PerspectiveCamera::build_subscription() {
  std::string topic = "image_raw";
  if (!this->in_calib.cam_names.empty()) {
    const auto index = this->get_parameter("input.index").as_int();
    const auto topic_ = this->in_calib.cam_names[index];
    if (!topic_.empty()) {
      topic = topic_;
    }
  }
  this->sub = this->create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::SensorDataQoS(),
      std::bind(&PerspectiveCamera::image_callback, this,
                std::placeholders::_1));
}

} // namespace virtcam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(virtcam::PerspectiveCamera)
