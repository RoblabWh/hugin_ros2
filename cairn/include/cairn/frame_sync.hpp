#ifndef CAIRN__FRAME_SYNC_HPP_
#define CAIRN__FRAME_SYNC_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <utility>
#include <vector>

#include <cuvslam/cuvslam2.h>
#include <cv_bridge/cv_bridge.hpp>

namespace cairn {
// Approximate-time synchroniser over a runtime number of image streams.
class ImageSync {
public:
  using ImageConstPtr = cv_bridge::CvImageConstPtr;

  struct ImageMatch {
    size_t camera_index;
    int64_t stamp_ns;
    ImageConstPtr image;
  };

  using Callback =
      std::function<void(int64_t stamp_ns, const std::vector<ImageMatch> &)>;

  ImageSync(size_t num_cameras, int64_t threshold_ns, size_t min_num_images,
            size_t buffer_size);

  void registerCallback(Callback callback);

  void addImage(size_t camera_idx, int64_t timestamp_ns, ImageConstPtr image);

  void clear();

  size_t numCameras() const { return buffers_.size(); }

private:
  std::vector<int64_t> peekHeads() const;
  void matchStreams();

  int64_t threshold_ns_;
  size_t min_num_images_;
  size_t buffer_size_;
  std::vector<std::deque<std::pair<int64_t, ImageConstPtr>>> buffers_;
  Callback callback_;
};

// Staging queue for IMU samples between frames.
//
// cuVSLAM requires all IMU samples older than a frame to be registered before
// that frame, in timestamp order. This queue holds them until the frame
// arrives.
class ImuQueue {
public:
  explicit ImuQueue(size_t capacity);

  void push(const cuvslam::ImuMeasurement &measurement);

  // Pop all samples with timestamp_ns < before_ns, in order.
  std::vector<cuvslam::ImuMeasurement> popUntil(int64_t timestamp_ns);

  void clear();

  size_t size() const { return queue_.size(); }

  // Samples dropped because the queue was full.
  size_t dropped() const { return dropped_; }

private:
  size_t capacity_;
  size_t dropped_{0};
  std::deque<cuvslam::ImuMeasurement> queue_;
};

} // namespace cairn

#endif // CAIRN__FRAME_SYNC_HPP_
