#include "dai_vi_ros2/dai_vi_ros2.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#ifndef NDEBUG
#define RCLCPP_NDEBUG(logger, fmt, ...) RCLCPP_DEBUG(logger, fmt, __VA_ARGS__)
#else
#define RCLCPP_NDEBUG(logger, fmt, ...)
#endif

namespace {
class StdVectorAllocator : public cv::MatAllocator {
  std::vector<std::uint8_t> &buf_;

public:
  explicit StdVectorAllocator(std::vector<std::uint8_t> &buf) : buf_(buf) {}

  cv::UMatData *allocate(int dims, const int *sizes, int type, void * /*data0*/,
                         size_t *step, cv::AccessFlag /*flags*/,
                         cv::UMatUsageFlags /*usageFlags*/) const override {
    size_t total = CV_ELEM_SIZE(type);
    for (int i = dims - 1; i >= 0; --i) {
      if (step) {
        step[i] =
            (i == dims - 1) ? CV_ELEM_SIZE(type) : step[i + 1] * sizes[i + 1];
      }
      total *= sizes[i];
    }
    buf_.resize(total);
    auto *u = new cv::UMatData(this);
    u->data = u->origdata = buf_.data();
    u->size = total;
    u->flags |= cv::UMatData::USER_ALLOCATED;
    return u;
  }

  bool allocate(cv::UMatData *u, cv::AccessFlag,
                cv::UMatUsageFlags) const override {
    return u != nullptr;
  }

  void deallocate(cv::UMatData *u) const override {
    if (u == nullptr) {
      return;
    }
    delete u;
  }
};

  uint64_t read_sysfs(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open sysfs file: " + path);
    }
    uint64_t value;
    file >> value;
    if (!file) {
      throw std::runtime_error("Failed to read from sysfs file: " + path);
    }
    return value;
  }

  void write_sysfs(const std::string &path, uint64_t value) {
    std::ofstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open sysfs file: " + path);
    }
    file << value;
    file.close();
    if (!file) {
      throw std::runtime_error("Failed to write to sysfs file: " + path);
    }
  }
} // namespace

namespace dai_vi
{
  ROS2Wrapper::ROS2Wrapper(const rclcpp::NodeOptions &options)
      : Node("dai_vi", options)
  {
    // Handle ROS parameters

    // frame ids
    this->frame_prefix = std::string(this->get_name()) + "_";
    this->frame_imu = this->frame_prefix + "imu";

    // exposure in frame_id for TUMVI compatibility
    declare_parameter("exposure_in_frame_id", false);
    this->exposure_in_frame_id = get_parameter("exposure_in_frame_id").as_bool();

    // device selection
    declare_parameter("device_id", std::string(""));
    declare_parameter("usb_speed", 5);

    const auto device_id_ = get_parameter("device_id").as_string();
    const std::optional<std::string> device_id = device_id_.empty() ? std::nullopt : std::make_optional(device_id_);
    const auto usb_speed = static_cast<dai::UsbSpeed>(get_parameter("usb_speed").as_int());

    // imu
    declare_parameter("imu.hz", 0);
    const auto imu_hz = get_parameter("imu.hz").as_int();

    // camera general
    declare_parameter("cams", std::vector<int64_t>{0, 1, 2, 3});
    declare_parameter("sync.cams", std::vector<int64_t>{});
    declare_parameter("sync.type", static_cast<int>(SyncType::SOFTWARE));
    declare_parameter("sync.stamps", true);
    declare_parameter("sync.on_host", false);
    declare_parameter("sync.leon_css", false);
    declare_parameter("sync.pwmchip", "");

    const auto cams = get_parameter("cams").as_integer_array();
    const auto sync_cams = get_parameter("sync.cams").as_integer_array();
    const auto sync_type = static_cast<SyncType>(get_parameter("sync.type").as_int());
    const auto sync_stamps = get_parameter("sync.stamps").as_bool();
    const auto sync_host = get_parameter("sync.on_host").as_bool();
    const auto sync_css = get_parameter("sync.leon_css").as_bool();
    const auto sync_pwmchip = get_parameter("sync.pwmchip").as_string();
    try {
      const auto id = std::stoul(sync_pwmchip);
      sysfs_pwmchip = "/sys/class/pwm/pwmchip" + std::to_string(id);
    } catch (const std::invalid_argument &) {
      RCLCPP_DEBUG(this->get_logger(), "sync.pwmchip value '%s' is not an integer, using as path", sync_pwmchip.c_str());
      sysfs_pwmchip = sync_pwmchip;
    }

    // camera defaults
    declare_parameter("cam.hz", 20.0f);
    declare_parameter("cam.width", 0);
    declare_parameter("cam.height", 0);
    declare_parameter("cam.color", true);
    declare_parameter("cam.exposure", 0);
    declare_parameter("cam.iso", 100);
    declare_parameter("cam.encode", -1);
    declare_parameter("cam.warmup", 0);

    const auto cam_hz = get_parameter("cam.hz").as_double();
    const auto cam_width = get_parameter("cam.width").as_int();
    const auto cam_height = get_parameter("cam.height").as_int();
    const auto cam_color = get_parameter("cam.color").as_bool();
    const auto cam_exposure = get_parameter("cam.exposure").as_int();
    const auto cam_iso = get_parameter("cam.iso").as_int();
    const auto cam_encode = get_parameter("cam.encode").as_int();
    const auto cam_warmup = get_parameter("cam.warmup").as_int();

    // camera per instance
    for (const auto camid : cams)
    {
      const auto name = "cam" + std::to_string(camid);
      this->frame_cam[name] = this->frame_prefix + name + "_optical_frame";
      declare_parameter(name + ".hz", cam_hz);
      declare_parameter(name + ".width", cam_width);
      declare_parameter(name + ".height", cam_height);
      declare_parameter(name + ".color", cam_color);
      declare_parameter(name + ".exposure", cam_exposure);
      declare_parameter(name + ".iso", cam_iso);
      declare_parameter(name + ".encode", cam_encode);
      declare_parameter(name + ".warmup", cam_warmup);
    }

    // Detect ROS time offset to align with DAI timestamps
    auto ros_time = get_clock()->now();
    auto steady_time = std::chrono::steady_clock::now();
    time_offset = std::chrono::nanoseconds(ros_time.nanoseconds() - steady_time.time_since_epoch().count());

    // Setup DAI-VI sensor
    sensor = std::make_unique<dai_vi::SensorWrapper>(device_id, usb_speed);

    if (imu_hz > 0)
    {
      sensor->addIMU(imu_hz);
      sensor->resetIMUCallback(std::bind(&ROS2Wrapper::publish_imu, this, std::placeholders::_1));

      pub_imu = create_publisher<sensor_msgs::msg::Imu>("~/imu/data_raw", rclcpp::SensorDataQoS());
    }

    if (cam_hz > 0)
    {
      for (const auto camid : cams) {
        const auto name = "cam" + std::to_string(camid);

        // Parse all parameters
        const auto width = get_parameter(name + ".width").as_int();
        const auto height = get_parameter(name + ".height").as_int();
        std::optional<std::pair<uint32_t, uint32_t>> resolution;
        if (width > 0 && height > 0) {
          resolution = {width, height};
        } else if (width > 0) {
          RCLCPP_WARN(this->get_logger(), "%s: width is set but height is not, ignoring width.", name.c_str());
        } else if (height > 0) {
          RCLCPP_WARN(this->get_logger(), "%s: height is set but width is not, ignoring height.", name.c_str());
        }
        const auto hz = get_parameter(name + ".hz").as_double();
        const auto exposure_ = get_parameter(name + ".exposure").as_int();
        const auto iso_ = get_parameter(name + ".iso").as_int();
        std::optional<std::chrono::microseconds> exposure;
        std::optional<uint32_t> iso;
        if (exposure_ > 0) {
          exposure = std::chrono::microseconds(exposure_);
          iso = iso_;
        } else if (iso_ != 100) {
          RCLCPP_WARN(this->get_logger(), "%s: ISO is set but exposure is not, ignoring ISO.", name.c_str());
        }
        const auto color = get_parameter(name + ".color").as_bool();
        const auto encode_ = get_parameter(name + ".encode").as_int();
        std::optional<uint32_t> encode;
        if (encode_ >= 0) {
          encode = encode_;
          if (encode_ > 100) {
            RCLCPP_WARN(this->get_logger(), "%s: JPEG quality is out of range 0 - 100", name.c_str());
          }
        }
        const auto warmup = get_parameter(name + ".warmup").as_int();

        // Add camera to sensor wrapper
        auto cam_added = sensor->addCamera(name, static_cast<dai::CameraBoardSocket>(camid), resolution, hz, exposure, iso, color, encode, warmup);
        if (!cam_added) {
          throw std::invalid_argument("Failed to create camera!");
        }

        if (std::find(sync_cams.begin(), sync_cams.end(), camid) != sync_cams.end()) {
          sensor->sync_cams.insert(name);
        }

        pub_cam[name] = create_publisher<sensor_msgs::msg::Image>("~/" + name + "/image_raw", rclcpp::SensorDataQoS());
        pub_cam_meta[name] = create_publisher<realsense2_camera_msgs::msg::Metadata>("~/" + name + "/metadata", rclcpp::SensorDataQoS());
      }

      sensor->sync_type = sync_type;
      sensor->sync_host = sync_host;
      sensor->sync_proc = sync_css ? dai::ProcessorType::LEON_CSS : dai::ProcessorType::LEON_MSS;
      sensor->sync_stamps = sync_stamps;
      sensor->resetCamCallback(std::bind(&ROS2Wrapper::publish_img, this, std::placeholders::_1, std::placeholders::_2));
    }

    if (!sensor->buildPipeline())
      throw std::invalid_argument("Failed to build pipeline");

    if (sync_type == SyncType::EXTERNAL && !sysfs_pwmchip.empty() && !sync_cams.empty()) {
      const auto sync_hz = sensor->node_cam[*sensor->sync_cams.begin()]->getMaxRequestedFps();
      const auto sync_period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / sync_hz));
      const auto sync_duty = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(1));

      if (std::filesystem::exists(sysfs_pwmchip + "/pwm0")) {
        throw std::runtime_error(sysfs_pwmchip + " is already in use");
      }
      write_sysfs(sysfs_pwmchip + "/export", 0);

      // Wait for the sysfs files to be writable
      for (uint16_t i = 0; i < 50; ++i) {
        if (std::ofstream(sysfs_pwmchip + "/pwm0/enable") &&
            std::ofstream(sysfs_pwmchip + "/pwm0/period") &&
            std::ofstream(sysfs_pwmchip + "/pwm0/duty_cycle")) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      // Cleanup remnants
      if (read_sysfs(sysfs_pwmchip + "/pwm0/period") != 0) {
        write_sysfs(sysfs_pwmchip + "/pwm0/enable", 0);
        write_sysfs(sysfs_pwmchip + "/pwm0/duty_cycle", 0);
      }
      // Configure PWM
      write_sysfs(sysfs_pwmchip + "/pwm0/period", sync_period.count());
      write_sysfs(sysfs_pwmchip + "/pwm0/duty_cycle", sync_duty.count());
      write_sysfs(sysfs_pwmchip + "/pwm0/enable", 1);
    }

    sensor->start();
  }

  ROS2Wrapper::~ROS2Wrapper() {
    sensor->stop();

    if (sensor->sync_type == SyncType::EXTERNAL && !sysfs_pwmchip.empty()) {
      write_sysfs(sysfs_pwmchip + "/pwm0/enable", 0);
      write_sysfs(sysfs_pwmchip + "/unexport", 0);
    }
  }

  void ROS2Wrapper::publish_img(std::shared_ptr<dai::ImgFrame> img, const std::string &name) {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time((img->getTimestamp().time_since_epoch() + time_offset).count());

    if (this->exposure_in_frame_id) {
      header.frame_id = std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(img->getExposureTime()).count());
    } else {
      header.frame_id = this->frame_cam[name];
    }

    auto &pub_meta = pub_cam_meta[name];
    if (pub_meta->get_subscription_count() > 0) {
      auto meta_msg = std::make_unique<realsense2_camera_msgs::msg::Metadata>();
      meta_msg->header = header;

      std::ostringstream json;
      json << std::fixed << "{"
           << "\"frame_number\":" << img->getSequenceNum()
           << ",\"clock_domain\":" << "\"hardware_clock\""
           << ",\"frame_timestamp\":"
           << std::chrono::duration<double, std::milli>(
                  img->getTimestampDevice().time_since_epoch())
                  .count()
           << ",\"actual_exposure\":" << img->getExposureTime().count()
           << ",\"sensitivity_iso\":" << img->getSensitivity()
           << ",\"sensor_mode\":" << img->getSensorMode();
      if (img->getColorTemperature() != 0) {
        json << ",\"white_balance\":" << img->getColorTemperature();
      }
      if (img->getLensPosition() != -1) {
        json << ",\"lens_position\":" << img->getLensPosition();
      }
      if (img->getFsync() != dai::ImgFrame::Fsync::NONE) {
        json << ",\"fsync\":" << static_cast<int>(img->getFsync());
      }
      json << "}";
      meta_msg->json_data = json.str();

      pub_meta->publish(std::move(meta_msg));
    }

    auto &pub_img = pub_cam[name];
    if (pub_img->get_subscription_count() > 0) {
      RCLCPP_NDEBUG(this->get_logger(), "<publish_images> stamp=%f dai=%p", rclcpp::Time(header.stamp).seconds(), img->getData().data());

      auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
      StdVectorAllocator alloc(img_msg->data);
      cv::Mat cv_img;
      cv_img.allocator = &alloc;
      if (img->getType() == dai::ImgFrame::Type::BITSTREAM) {
        const auto bitstream = img->getFrame(false);
        if (bitstream.empty()) {
          RCLCPP_ERROR(this->get_logger(), "Received empty bitstream for camera %s", name.c_str());
          return;
        }
        cv::imdecode(bitstream, cv::IMREAD_UNCHANGED, &cv_img);
      } else {
        cv_img = img->getCvFrame(&alloc);
      }

      img_msg->header = header;
      img_msg->height = cv_img.rows;
      img_msg->width = cv_img.cols;
      img_msg->step = cv_img.step;
      img_msg->encoding = cv_img.channels() == 3 ? "bgr8" : "mono8";

      RCLCPP_NDEBUG(this->get_logger(), "<publish_images> stamp=%f msg=%p", rclcpp::Time(header.stamp).seconds(), img_msg->data.data());
      pub_img->publish(std::move(img_msg));
    }
  }

  void ROS2Wrapper::publish_imu(const dai::IMUPacket &pkt) {
    const auto &accel = pkt.acceleroMeter;
    const auto &gyro = pkt.gyroscope;

    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->header.stamp = rclcpp::Time((accel.getTimestamp().time_since_epoch() + time_offset).count());
    msg->header.frame_id = this->frame_imu;

    msg->linear_acceleration.x = accel.x;
    msg->linear_acceleration.y = accel.y;
    msg->linear_acceleration.z = accel.z;

    msg->angular_velocity.x = gyro.x;
    msg->angular_velocity.y = gyro.y;
    msg->angular_velocity.z = gyro.z;

    RCLCPP_NDEBUG(this->get_logger(), "<publish_imu> stamp=%f address=%p", rclcpp::Time(msg->header.stamp).seconds(), static_cast<void *>(msg.get()));
    pub_imu->publish(std::move(msg));
  }

} // namespace dai_vi

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dai_vi::ROS2Wrapper)
