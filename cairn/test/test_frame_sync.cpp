#include <vector>

#include <gtest/gtest.h>

#include "cairn/frame_sync.hpp"

namespace {

constexpr int64_t kMs = 1000000;

cairn::ImageSync::ImageConstPtr make_image() {
  return std::make_shared<const cv_bridge::CvImage>();
}

struct Recorder {
  std::vector<int64_t> stamps;
  std::vector<std::vector<size_t>> camera_sets;

  void operator()(int64_t stamp_ns,
                  const std::vector<cairn::ImageSync::ImageMatch> &set) {
    stamps.push_back(stamp_ns);
    std::vector<size_t> cameras;
    for (const auto &match : set) {
      cameras.push_back(match.camera_index);
    }
    camera_sets.push_back(cameras);
  }
};

} // namespace

TEST(ImageSync, FiresWhenBothCamerasArriveWithinThreshold) {
  cairn::ImageSync sync(2, kMs, 2, 10);
  Recorder recorder;
  sync.registerCallback([&recorder](auto s, const auto &m) { recorder(s, m); });

  sync.addImage(0, 1000, make_image());
  EXPECT_TRUE(recorder.stamps.empty()) << "must wait for the second camera";

  sync.addImage(1, 1000 + kMs / 2, make_image());
  ASSERT_EQ(recorder.stamps.size(), 1u);
  EXPECT_EQ(recorder.stamps[0], 1000 + kMs / 2)
      << "callback carries the newest stamp of the set";
  EXPECT_EQ(recorder.camera_sets[0], (std::vector<size_t>{0, 1}));
}

TEST(ImageSync, MatchCarriesPerImageTimestamps) {
  cairn::ImageSync sync(2, kMs, 2, 10);
  std::vector<int64_t> stamps;
  sync.registerCallback(
      [&stamps](int64_t, const std::vector<cairn::ImageSync::ImageMatch> &set) {
        for (const auto &match : set) {
          stamps.push_back(match.stamp_ns);
        }
      });

  // cuVSLAM keeps per-image stamps and validates the spread itself, so the
  // group stamp must not be substituted for the individual ones.
  sync.addImage(0, 5000, make_image());
  sync.addImage(1, 5500, make_image());
  ASSERT_EQ(stamps.size(), 2u);
  EXPECT_EQ(stamps[0], 5000);
  EXPECT_EQ(stamps[1], 5500);
}

TEST(ImageSync, DropsSetOutsideThresholdInsteadOfWedging) {
  cairn::ImageSync sync(2, kMs, 2, 10);
  Recorder recorder;
  sync.registerCallback([&recorder](auto s, const auto &m) { recorder(s, m); });

  // Camera 0 runs 5 ms ahead of the threshold: the stale head is discarded, not
  // retained.
  sync.addImage(0, 0, make_image());
  sync.addImage(1, 5 * kMs, make_image());
  EXPECT_TRUE(recorder.stamps.empty());

  // The next camera-0 frame lines up with the camera-1 frame still queued.
  sync.addImage(0, 5 * kMs + 100, make_image());
  ASSERT_EQ(recorder.stamps.size(), 1u);
  EXPECT_EQ(recorder.camera_sets[0], (std::vector<size_t>{0, 1}));
}

TEST(ImageSync, PartialSetFiresWhenMinNumImagesAllowsIt) {
  cairn::ImageSync sync(3, kMs, /*min_num_images=*/2, 10);
  Recorder recorder;
  sync.registerCallback([&recorder](auto s, const auto &m) { recorder(s, m); });

  // Cameras 0 and 1 are aligned, with camera 2 far behind. Matching still needs
  // every stream to have something pending, so give camera 2 an old frame.
  sync.addImage(2, 0, make_image());
  sync.addImage(0, 10 * kMs, make_image());
  sync.addImage(1, 10 * kMs, make_image());

  // The first pass pops camera 2's stale head alone (1 < min_num_images, so no
  // callback), then the second pass has 0 and 1 aligned but camera 2 empty ->
  // waits.
  EXPECT_TRUE(recorder.stamps.empty());

  sync.addImage(2, 10 * kMs + 200, make_image());
  ASSERT_EQ(recorder.stamps.size(), 1u);
  EXPECT_EQ(recorder.camera_sets[0], (std::vector<size_t>{0, 1, 2}));
}

TEST(ImageSync, BufferOverflowDropsOldest) {
  cairn::ImageSync sync(2, kMs, 2, /*buffer_size=*/2);
  Recorder recorder;
  sync.registerCallback([&recorder](auto s, const auto &m) { recorder(s, m); });

  // Camera 0 publishes three frames while camera 1 is silent, so the oldest is
  // evicted.
  sync.addImage(0, 1 * kMs * 100, make_image());
  sync.addImage(0, 2 * kMs * 100, make_image());
  sync.addImage(0, 3 * kMs * 100, make_image());
  EXPECT_TRUE(recorder.stamps.empty());

  sync.addImage(1, 2 * kMs * 100, make_image());
  ASSERT_EQ(recorder.stamps.size(), 1u);
  EXPECT_EQ(recorder.stamps[0], 2 * kMs * 100)
      << "the evicted frame cannot be the match";
}

TEST(ImageSync, RejectsOutOfRangeCameraIndex) {
  cairn::ImageSync sync(2, kMs, 2, 10);
  EXPECT_THROW(sync.addImage(2, 0, make_image()), std::out_of_range);
}

TEST(ImuQueue, DrainsStrictlyOlderSamplesInOrder) {
  cairn::ImuQueue queue(10);
  for (int64_t t : {100, 200, 300, 400}) {
    cuvslam::ImuMeasurement m{};
    m.timestamp_ns = t;
    queue.push(m);
  }

  const auto drained = queue.popUntil(300);
  ASSERT_EQ(drained.size(), 2u);
  EXPECT_EQ(drained[0].timestamp_ns, 100);
  EXPECT_EQ(drained[1].timestamp_ns, 200);
  EXPECT_EQ(queue.size(), 2u)
      << "the sample exactly at the boundary stays for the next frame";

  EXPECT_TRUE(queue.popUntil(300).empty());
  EXPECT_EQ(queue.popUntil(1000).size(), 2u);
  EXPECT_EQ(queue.size(), 0u);
}

TEST(ImuQueue, CountsDropsOnOverflow) {
  cairn::ImuQueue queue(2);
  for (int64_t t : {1, 2, 3, 4}) {
    cuvslam::ImuMeasurement m{};
    m.timestamp_ns = t;
    queue.push(m);
  }
  EXPECT_EQ(queue.size(), 2u);
  EXPECT_EQ(queue.dropped(), 2u);

  const auto drained = queue.popUntil(1000);
  ASSERT_EQ(drained.size(), 2u);
  EXPECT_EQ(drained[0].timestamp_ns, 3)
      << "oldest samples are the ones dropped";
}
