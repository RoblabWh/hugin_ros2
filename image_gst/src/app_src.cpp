#include "image_gst/app_src.hpp"

#include <cstdint>
#include <gst/video/video.h>

#include <stdexcept>
#include <string>
#include <unordered_map>

#include <rclcpp/logging.hpp>

namespace image_gst {

AppSrc::AppSrc(const rclcpp::NodeOptions &options)
    : Node("gst_app_src", options) {
  const auto desc = this->declare_parameter<std::string>("pipeline", "");
  if (desc.empty()) {
    RCLCPP_ERROR(this->get_logger(), "parameter 'pipeline' is required");
    throw std::runtime_error("parameter 'pipeline' is required");
  }

  // Initialize GStreamer
  GError *err = nullptr;
  if (!gst_init_check(nullptr, nullptr, &err)) {
    const std::string what = err ? err->message : "unknown error";
    g_clear_error(&err);
    throw std::runtime_error("gst_init failed: " + what);
  }

  // Parse gstreamer pipeline
  this->pipeline_ = gst_parse_launch(desc.c_str(), &err);
  if (!this->pipeline_) {
    const std::string what = err ? err->message : "unknown error";
    g_clear_error(&err);
    throw std::runtime_error("failed to parse pipeline: " + what);
  }
  if (err) {
    RCLCPP_WARN(this->get_logger(), "while parsing pipeline: %s", err->message);
    g_clear_error(&err);
  }

  // Check that appsrc is part of the pipeline
  GstIterator *iter = GST_IS_BIN(this->pipeline_)
                          ? gst_bin_iterate_recurse(GST_BIN(this->pipeline_))
                          : nullptr;
  int found = 0;
  if (iter) {
    GValue item = G_VALUE_INIT;
    while (gst_iterator_next(iter, &item) == GST_ITERATOR_OK) {
      auto *element = GST_ELEMENT(g_value_get_object(&item));
      if (GST_IS_APP_SRC(element)) {
        this->source_ = element;
        ++found;
      }
      g_value_unset(&item);
    }
    gst_iterator_free(iter);
  }
  if (found != 1) {
    throw std::runtime_error("pipeline must contain exactly one appsrc");
  }

  // Force needed appsrc config for timing invariants
  g_object_set(this->source_, "format", GST_FORMAT_TIME, "do-timestamp", FALSE,
               "is-live", TRUE, nullptr);

  this->bus_ = gst_element_get_bus(this->pipeline_);

  if (gst_element_set_state(this->pipeline_, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE) {
    throw std::runtime_error("failed to start pipeline");
  }

  this->sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      std::bind(&AppSrc::callback, this, std::placeholders::_1));

  this->bus_thread_ = std::thread(&AppSrc::busLoop, this);
}

AppSrc::~AppSrc() {
  if (this->pipeline_) {
    gst_element_send_event(this->pipeline_, gst_event_new_eos());
    gst_element_set_state(this->pipeline_, GST_STATE_NULL);
    gst_object_unref(this->pipeline_);
  }
  if (this->caps_) {
    gst_caps_unref(this->caps_);
  }

  gst_bus_post(this->bus_, gst_message_new_application(
                               nullptr, gst_structure_new_empty("quit")));
  this->bus_thread_.join();
  gst_object_unref(this->bus_);
}

void AppSrc::busLoop() {
  while (GstMessage *m = gst_bus_timed_pop_filtered(
             this->bus_, GST_CLOCK_TIME_NONE,
             static_cast<GstMessageType>(GST_MESSAGE_ERROR |
                                         GST_MESSAGE_WARNING | GST_MESSAGE_EOS |
                                         GST_MESSAGE_APPLICATION))) {
    GError *err = nullptr;
    gchar *debug = nullptr;
    switch (GST_MESSAGE_TYPE(m)) {
    case GST_MESSAGE_ERROR:
      gst_message_parse_error(m, &err, &debug);
      RCLCPP_ERROR(this->get_logger(), "%s: %s", GST_OBJECT_NAME(m->src),
                   err->message);
      break;
    case GST_MESSAGE_WARNING:
      gst_message_parse_warning(m, &err, &debug);
      RCLCPP_WARN(this->get_logger(), "%s: %s", GST_OBJECT_NAME(m->src),
                  err->message);
      break;
    case GST_MESSAGE_APPLICATION: // shutdown sentinel
      gst_message_unref(m);
      return;
    default:
      RCLCPP_WARN(this->get_logger(), "pipeline reached end of stream");
      break;
    }
    g_clear_error(&err);
    g_free(debug);
    gst_message_unref(m);
  }
}

void AppSrc::callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {

  // Subset of encodings supporting zero-copy, with gstreamer mappings
  static const std::unordered_map<std::string, GstVideoFormat> formats = {
      {"bgr8", GST_VIDEO_FORMAT_BGR},
      {"rgb8", GST_VIDEO_FORMAT_RGB},
      {"bgra8", GST_VIDEO_FORMAT_BGRA},
      {"rgba8", GST_VIDEO_FORMAT_RGBA},
      {"mono8", GST_VIDEO_FORMAT_GRAY8},
      {"uyvy", GST_VIDEO_FORMAT_UYVY},
      {"yuyv", GST_VIDEO_FORMAT_YUY2},
      {"mono16", GST_VIDEO_FORMAT_GRAY16_LE},
      {"nv21", GST_VIDEO_FORMAT_NV21},
      {"nv24", GST_VIDEO_FORMAT_NV24},
      // sensor_msgs is missing these, add them anyway for convenience
      {"nv12", GST_VIDEO_FORMAT_NV12},
      {"nv16", GST_VIDEO_FORMAT_NV16},
      {"i420", GST_VIDEO_FORMAT_I420},
      {"yvyu", GST_VIDEO_FORMAT_YVYU},
      // sensor_msgs deprecates these two in favour of uyvy and yuyv
      {"yuv422", GST_VIDEO_FORMAT_UYVY},
      {"yuv422_yuy2", GST_VIDEO_FORMAT_YUY2},
  };

  // Determine the gstreamer format from the ROS encoding
  const auto entry = formats.find(msg->encoding);
  if (entry == formats.end()) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "unsupported encoding '%s'", msg->encoding.c_str());
    return;
  }
  GstVideoFormat format = entry->second;
  if (msg->is_bigendian && format == GST_VIDEO_FORMAT_GRAY16_LE) {
    format = GST_VIDEO_FORMAT_GRAY16_BE;
  }

  // Handle caps negotiation
  GstCaps *caps =
      gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING,
                          gst_video_format_to_string(format), "width",
                          G_TYPE_INT, static_cast<int>(msg->width), "height",
                          G_TYPE_INT, static_cast<int>(msg->height), nullptr);
  if (!this->caps_ || !gst_caps_is_equal(this->caps_, caps)) {
    gchar *text = gst_caps_to_string(caps);
    RCLCPP_INFO(this->get_logger(), "negotiating caps: %s", text);
    g_free(text);
    gst_app_src_set_caps(GST_APP_SRC(this->source_), caps);
    if (this->caps_) {
      gst_caps_unref(this->caps_);
    }
    this->caps_ = gst_caps_ref(caps);
  }
  gst_caps_unref(caps);

  // Reject invalid gstreamer geometry
  GstVideoInfo info;
  if (msg->width == 0 || msg->height == 0 ||
      !gst_video_info_set_format(&info, format, msg->width, msg->height)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "invalid geometry %ux%u for encoding '%s'",
                          msg->width, msg->height, msg->encoding.c_str());
    return;
  }

  // Construct gstreamer video meta to match ros format
  const guint planes = GST_VIDEO_INFO_N_PLANES(&info);
  const gint base = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);
  gsize offsets[GST_VIDEO_MAX_PLANES] = {0};
  gint strides[GST_VIDEO_MAX_PLANES] = {0};
  gsize expected = 0;
  for (guint i = 0; i < planes; ++i) {
    const gint stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, i);
    const gsize end = i + 1 < planes ? GST_VIDEO_INFO_PLANE_OFFSET(&info, i + 1)
                                     : GST_VIDEO_INFO_SIZE(&info);
    const gsize rows = (end - GST_VIDEO_INFO_PLANE_OFFSET(&info, i)) / stride;
    strides[i] = static_cast<gint>(msg->step) * stride / base;
    offsets[i] = expected;
    expected += static_cast<gsize>(strides[i]) * rows;
  }

  // Reject buffers with mismatched geometry
  if (msg->step <
          static_cast<gsize>(GST_VIDEO_FORMAT_INFO_PSTRIDE(info.finfo, 0)) *
              msg->width ||
      msg->data.size() < expected) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "message geometry does not match payload: step=%u "
                          "%ux%u '%s' needs %zu bytes, got %zu",
                          msg->step, msg->width, msg->height,
                          msg->encoding.c_str(), expected, msg->data.size());
    return;
  }

  // Zero copy: The destroy notify holds the ROS message alive for exactly as
  // long as GStreamer references it
  auto *ownership = new sensor_msgs::msg::Image::ConstSharedPtr(msg);
  GstBuffer *buffer = gst_buffer_new_wrapped_full(
      GST_MEMORY_FLAG_READONLY, const_cast<uint8_t *>(msg->data.data()),
      msg->data.size(), 0, msg->data.size(), ownership, [](gpointer p) {
        delete static_cast<sensor_msgs::msg::Image::ConstSharedPtr *>(p);
      });

  gst_buffer_add_video_meta_full(buffer, GST_VIDEO_FRAME_FLAG_NONE, format,
                                 msg->width, msg->height, planes, offsets,
                                 strides);

  // Calculate pts from ros timestamp with offset to anchor onto the pipeline
  // running time
  int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (stamp == 0) {
    stamp = this->now().nanoseconds();
  }
  if (!this->offset_ || stamp - *this->offset_ < this->last_pts_) {
    if (this->offset_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "source timestamps jumped backwards, re-anchoring");
    }
    const GstClockTime running =
        gst_element_get_current_running_time(this->pipeline_);
    this->offset_ =
        stamp -
        (GST_CLOCK_TIME_IS_VALID(running) ? static_cast<int64_t>(running) : 0);
  }
  this->last_pts_ = stamp - *this->offset_;
  GST_BUFFER_PTS(buffer) = static_cast<GstClockTime>(this->last_pts_);
  GST_BUFFER_DURATION(buffer) = GST_CLOCK_TIME_NONE; // variable rate

  // Push buffer into pipeline
  const GstFlowReturn ret =
      gst_app_src_push_buffer(GST_APP_SRC(this->source_), buffer);
  if (ret != GST_FLOW_OK) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "failed to push buffer: %s", gst_flow_get_name(ret));
  }
}

} // namespace image_gst

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(image_gst::AppSrc)
