#include "cairn/frame_sync.hpp"

#include <algorithm>
#include <stdexcept>

namespace cairn {

ImageSync::ImageSync(size_t num_cameras, int64_t threshold_ns,
                     size_t min_num_images, size_t buffer_size)
    : threshold_ns_(threshold_ns),
      min_num_images_(std::min(min_num_images, num_cameras)),
      buffer_size_(std::max<size_t>(buffer_size, 1)), buffers_(num_cameras) {
  if (num_cameras == 0) {
    throw std::invalid_argument("ImageSync needs at least one camera");
  }
}

void ImageSync::registerCallback(Callback callback) {
  callback_ = std::move(callback);
}

void ImageSync::clear() {
  for (auto &buffer : buffers_) {
    buffer.clear();
  }
}

void ImageSync::addImage(size_t camera_index, int64_t timestamp_ns,
                         ImageConstPtr image) {
  if (camera_index >= buffers_.size()) {
    throw std::out_of_range("camera index out of range");
  }

  auto &buffer = buffers_[camera_index];
  if (buffer.size() >= buffer_size_) {
    buffer.pop_front();
  }
  buffer.emplace_back(timestamp_ns, std::move(image));

  matchStreams();
}

std::vector<int64_t> ImageSync::peekHeads() const {
  std::vector<int64_t> heads;
  heads.reserve(buffers_.size());
  for (const auto &buffer : buffers_) {
    if (buffer.empty()) {
      return {};
    }
    heads.push_back(buffer.front().first);
  }
  return heads;
}

void ImageSync::matchStreams() {
  while (true) {
    const std::vector<int64_t> heads = peekHeads();
    if (heads.empty()) {
      return;
    }

    const int64_t min_stamp = *std::min_element(heads.begin(), heads.end());

    std::vector<size_t> matched;
    matched.reserve(heads.size());
    for (size_t i = 0; i < heads.size(); ++i) {
      if (heads[i] <= min_stamp + threshold_ns_) {
        matched.push_back(i);
      }
    }

    if (matched.size() >= min_num_images_ && callback_) {
      std::vector<ImageMatch> set;
      set.reserve(matched.size());
      int64_t newest = min_stamp;
      for (const size_t i : matched) {
        const auto &head = buffers_[i].front();
        newest = std::max(newest, head.first);
        set.push_back(ImageMatch{i, head.first, head.second});
      }
      callback_(newest, set);
    }

    for (const size_t i : matched) {
      buffers_[i].pop_front();
    }
  }
}

ImuQueue::ImuQueue(size_t capacity)
    : capacity_(std::max<size_t>(capacity, 1)) {}

void ImuQueue::push(const cuvslam::ImuMeasurement &measurement) {
  if (queue_.size() >= capacity_) {
    queue_.pop_front();
    ++dropped_;
  }
  queue_.push_back(measurement);
}

std::vector<cuvslam::ImuMeasurement> ImuQueue::popUntil(int64_t before_ns) {
  std::vector<cuvslam::ImuMeasurement> out;
  while (!queue_.empty() && queue_.front().timestamp_ns < before_ns) {
    out.push_back(queue_.front());
    queue_.pop_front();
  }
  return out;
}

void ImuQueue::clear() { queue_.clear(); }

} // namespace cairn
