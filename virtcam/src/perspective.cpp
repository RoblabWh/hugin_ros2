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

  this->declare_parameter("output.type", "pinhole");
  this->declare_parameter("output.res_x", 1280);
  this->declare_parameter("output.res_y", 720);
  this->declare_parameter("output.fov_x", 120.0);
  this->declare_parameter("output.fov_y", 0.0);
  this->declare_parameter("output.intrinsics", std::vector<double>{});

  this->declare_parameter("output.attach_to", std::vector<int>{0});
  this->declare_parameter("output.attach_offset", 0.5);
  this->declare_parameter("output.position",
                          std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("output.orientation",
                          std::vector<double>{0.0, 0.0, 0.0, 1.0});

  this->declare_parameter("output.frame_id", "virtual_camera");
  this->declare_parameter("frame.base", "base_link");
  this->declare_parameter("frame.imu", "imu");

  this->build_intrinsics();
  this->build_extrinsics();
  this->build_map();
  this->initialized = true;

  this->sub = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&PerspectiveCamera::image_callback, this, std::placeholders::_1));

  pub = this->create_publisher<sensor_msgs::msg::Image>(
      "virtcam/image_raw", rclcpp::SensorDataQoS());
}

void PerspectiveCamera::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &in_msg) {

  auto in_cvb = cv_bridge::toCvShare(in_msg);

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
    } else if (param.get_name() == "output.position") {
      const auto pos = param.as_double_array();
      if (pos.size() != 3) {
        result.successful = false;
        result.reason = "Position must be a 3-vector [x y z]";
        return result;
      }
    } else if (param.get_name() == "output.orientation") {
      const auto ori = param.as_double_array();
      if (ori.size() != 4) {
        result.successful = false;
        result.reason = "Orientation must be a quaternion (4-vector [x y z w])";
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
      } else if (fov_x >= 180.0) {
        result.successful = false;
        result.reason = "Horizontal FOV must be in (0, 180) degrees";
        return result;
      }
    } else if (param.get_name() == "output.fov_y") {
      const auto fov_y = param.as_double();
      if (fov_y <= 0.0) {
        placeholder_count++;
      } else if (fov_y >= 180.0) {
        result.successful = false;
        result.reason = "Vertical FOV must be in (0, 180) degrees";
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
  bool build_intr = false, build_extr = false, build_map = false;

  for (auto &param : params) {
    if (param.get_name() == "input.calibration") {
      const auto path = param.as_string();
      std::ifstream file(path);
      cereal::JSONInputArchive archive(file);
      archive(this->in_calib);
      build_map = true;
    } else if (param.get_name() == "input.index" ||
               param.get_name() == "input.depth") {
      build_map = true;
    } else if (param.get_name() == "output.attach_to" ||
               param.get_name() == "output.attach_offset" ||
               param.get_name() == "output.position" ||
               param.get_name() == "output.orientation" ||
               param.get_name() == "frame.base" ||
               param.get_name() == "frame.imu") {
      build_extr = true;
    } else if (param.get_name() == "output.type") {
      const auto type = param.as_string();
      this->out_intr = this->out_intr.fromString(type);
      build_intr = true;
    } else if (param.get_name() == "output.res_x" ||
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
  }
}

void PerspectiveCamera::build_intrinsics() {
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
  this->map.create(res_y, res_x, CV_32FC2);

  if (fov_x <= 0.0) {
    fov_x = fov_y * aspect;
  } else if (fov_y <= 0.0) {
    fov_y = fov_x / aspect;
  }

  const auto cx = res_x / 2.0;
  const auto cy = res_y / 2.0;

  const auto fx = cx / std::tan(fov_x * 0.5 * M_PI / 180.0);
  const auto fy = cy / std::tan(fov_y * 0.5 * M_PI / 180.0);

  this->out_intr.setFromInit(Eigen::Vector4f(fx, fy, cx, cy));

  // Handle additional intrinsics as good as we can
  const auto intr_extra =
      this->get_parameter("output.intrinsics").as_double_array();
  const auto intr_num = this->out_intr.getN();
  Eigen::VectorXf intr_inc = Eigen::VectorXf::Zero(intr_num);
  for (std::size_t i = 0;
       i < std::min<std::size_t>(intr_extra.size(), intr_num - 4); ++i) {
    intr_inc[i + 4] = intr_extra[i] - this->out_intr.getParam()[i + 4];
  }
  this->out_intr.applyInc(intr_inc);
}

void PerspectiveCamera::build_extrinsics() {
  const auto pos = this->get_parameter("output.position").as_double_array();
  const auto ori = this->get_parameter("output.orientation").as_double_array();

  this->out_extr.translation() = Eigen::Vector3f(pos[0], pos[1], pos[2]);
  this->out_extr.setQuaternion(
      Eigen::Quaternionf(ori[3], ori[0], ori[1], ori[2]));

  const auto attach_to =
      this->get_parameter("output.attach_to").as_integer_array();
  const auto attach_offset =
      this->get_parameter("output.attach_offset").as_double();
  if (attach_to.size() == 1) {
    this->out_extr = this->in_calib.T_i_c[attach_to[0]] * this->out_extr;
  } else if (attach_to.size() == 2) {
    const auto &t1 = this->in_calib.T_i_c[attach_to[0]];
    const auto &t2 = this->in_calib.T_i_c[attach_to[1]];

    const auto ti = t1.translation() +
                    (t2.translation() - t1.translation()) * attach_offset;
    const auto ri =
        t1.so3() * t1.so3().exp((t1.so3().inverse() * t2.so3()).log() * 0.5);
    // TODO above and below do nearly the same
    //  const auto interpolated = t1 * t1.exp((t1.inverse() * t2).log() * 0.5);

    this->out_extr = Sophus::SE3f(ri, ti) * this->out_extr;
  } else {
    const auto frame_base = this->get_parameter("frame.base").as_string();
    const auto frame_imu = this->get_parameter("frame.imu").as_string();

    Sophus::SE3f tf_imu_base;
    try {
      const auto tf = this->tf_buffer->lookupTransform(frame_imu, frame_base,
                                                       tf2::TimePointZero);
      tf_imu_base =
          Sophus::SE3f(tf2::transformToEigen(tf).matrix().cast<float>());
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Could not get transform between %s and %s: %s",
                  frame_base.c_str(), frame_imu.c_str(), ex.what());
    }
    this->out_extr = tf_imu_base *
                     Sophus::SE3f(Eigen::Quaternionf(0.5, -0.5, 0.5, -0.5),
                                  Eigen::Vector3f::Zero()) *
                     this->out_extr;
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
              map.forEach<cv::Vec2f>([&](auto &mapping, auto pos) -> void {
                Eigen::Vector3f p3d;
                auto p2d = Eigen::Map<Eigen::Vector2f>(mapping.val);

                bool good = out_cam.unproject(Eigen::Vector2f{pos[1], pos[0]}, p3d);
                if (good) {
                  p3d = in_extr.inverse() * out_extr * (p3d * static_cast<float>(in_depth));
                  good = in_cam.project(p3d, p2d);
                }
                if (!good) {
                  p2d.setZero();
                }
              });
            },
            out_intr.variant);
      },
      in_intr.variant);
}

} // namespace virtcam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(virtcam::PerspectiveCamera)
