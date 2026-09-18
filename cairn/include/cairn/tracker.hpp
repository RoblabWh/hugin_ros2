#ifndef CAIRN__TRACKER_HPP_
#define CAIRN__TRACKER_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <cuvslam/cuvslam2.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <foxglove_msgs/msg/image_annotations.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sophus/se3.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "cairn/calibration.hpp"
#include "cairn/frame_sync.hpp"

namespace cairn {

class Tracker : public rclcpp::Node {
public:
  explicit Tracker(const rclcpp::NodeOptions &options);
  ~Tracker() override;

private:
  void callbackImage(size_t cam_idx,
                     sensor_msgs::msg::Image::ConstSharedPtr msg);
  void callbackImu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void callbackSynced(int64_t stamp_ns,
                      const std::vector<ImageSync::ImageMatch> &set);

  RigOptions readRigOptions();
  Sophus::SE3d resolveRigFromReference(const std::string &reference_frame);

  void publishPose(int64_t stamp_ns,
                   const cuvslam::PoseWithCovariance &world_from_rig);
  void publishSlam(int64_t stamp_ns, const Sophus::SE3d &odom_from_rig);
  void publishDiagnostics(int64_t stamp_ns, bool tracking, double track_ms);
  void publishObservations(int64_t stamp_ns);
  void publishLandmarks(int64_t stamp_ns);

  basalt::Calibration<double> calib_;
  cuvslam::Rig rig_;
  size_t num_cameras_{0};
  int64_t cam_time_offset_ns_{0};
  bool imu_enabled_{false};

  std::string map_frame_;
  std::string odom_frame_;
  std::string rig_frame_;

  bool publish_odom_tf_{true};
  bool publish_map_tf_{true};
  bool enable_slam_{true};
  // Whether Odometry::Config asked for observation/landmark export.
  bool exports_enabled_{false};

  std::unique_ptr<cuvslam::Odometry> odometry_;
  std::unique_ptr<cuvslam::Slam> slam_;

  std::unique_ptr<ImageSync> sync_;
  std::unique_ptr<ImuQueue> imu_queue_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
      image_subs_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      status_pub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_pub_;
  std::deque<geometry_msgs::msg::PoseStamped> path_;
  int64_t last_path_publish_ns_{0};
  int64_t last_slam_path_publish_ns_{0};
  int64_t path_publish_period_ns_{0};

  // Debug views, advertised only with exports_enabled_.
  std::vector<
      rclcpp::Publisher<foxglove_msgs::msg::ImageAnnotations>::SharedPtr>
      observation_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr landmark_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool has_previous_{false};
  Sophus::SE3d previous_odom_from_rig_;
  int64_t previous_stamp_ns_{0};

  uint64_t frame_count_{0};
  uint64_t tracking_failures_{0};
};

} // namespace cairn

#endif // CAIRN__TRACKER_HPP_
