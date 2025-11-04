#ifndef VIRTCAM__SYNCHRONIZER_HPP_
#define VIRTCAM__SYNCHRONIZER_HPP_

#include "message_filters/subscriber.hpp"
#include "message_filters/time_synchronizer.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace virtcam {

class VariableSynchronizer {
  using MsgT = sensor_msgs::msg::Image;
  using MsgPtrT = MsgT::ConstSharedPtr;
  using SubT = message_filters::Subscriber<MsgT>;
  using VariantT = std::variant<
      std::monostate, message_filters::TimeSynchronizer<MsgT, MsgT>,
      message_filters::TimeSynchronizer<MsgT, MsgT, MsgT>,
      message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT>,
      message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT, MsgT>,
      message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT, MsgT, MsgT>,
      message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT, MsgT, MsgT,
                                        MsgT>,
      message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT, MsgT, MsgT,
                                        MsgT, MsgT>,
      message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT, MsgT, MsgT,
                                        MsgT, MsgT, MsgT>>;
  using UserCbT = std::function<void(const std::vector<MsgPtrT> &msgs)>;

public:
  VariableSynchronizer(rclcpp::Node *node,
                       const std::vector<std::string> &topics, UserCbT callback,
                       const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS())
      : node(node), user_cb(std::move(callback)) {
    sub.reset();
    subs.clear();
    if (topics.size() > 1) {
      subs.reserve(topics.size());
      for (const auto &topic : topics) {
        subs.emplace_back(
            std::make_unique<SubT>(node, topic, qos.get_rmw_qos_profile()));
      }
    }

    switch (topics.size()) {
    case 0:
      break;
    case 1: {
      this->sub = node->create_subscription<sensor_msgs::msg::Image>(
          topics.front(), qos,
          [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
            this->aggregate_cb(msg);
          });
      break;
    }
    case 2: {
      using SyncT = message_filters::TimeSynchronizer<MsgT, MsgT>;
      this->sync.emplace<SyncT>(*subs[0], *subs[1], qos.depth());
      std::get<SyncT>(this->sync)
          .registerCallback(std::bind(&VariableSynchronizer::helper2_cb, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));
      break;
    }
    case 3: {
      using SyncT = message_filters::TimeSynchronizer<MsgT, MsgT, MsgT>;
      this->sync.emplace<SyncT>(*subs[0], *subs[1], *subs[2], qos.depth());
      std::get<SyncT>(this->sync)
          .registerCallback(std::bind(
              &VariableSynchronizer::helper3_cb, this, std::placeholders::_1,
              std::placeholders::_2, std::placeholders::_3));
      break;
    }
    case 4: {
      using SyncT = message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT>;
      this->sync.emplace<SyncT>(*subs[0], *subs[1], *subs[2], *subs[3],
                                qos.depth());
      std::get<SyncT>(this->sync)
          .registerCallback(
              std::bind(&VariableSynchronizer::helper4_cb, this,
                        std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3, std::placeholders::_4));
      break;
    }
    case 5: {
      using SyncT =
          message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT, MsgT>;
      this->sync.emplace<SyncT>(*subs[0], *subs[1], *subs[2], *subs[3],
                                *subs[4], qos.depth());
      std::get<SyncT>(this->sync)
          .registerCallback(std::bind(
              &VariableSynchronizer::helper5_cb, this, std::placeholders::_1,
              std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5));
      break;
    }
    case 6: {
      using SyncT =
          message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT, MsgT, MsgT>;
      this->sync.emplace<SyncT>(*subs[0], *subs[1], *subs[2], *subs[3],
                                *subs[4], *subs[5], qos.depth());
      std::get<SyncT>(this->sync)
          .registerCallback(
              std::bind(&VariableSynchronizer::helper6_cb, this,
                        std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3, std::placeholders::_4,
                        std::placeholders::_5, std::placeholders::_6));
      break;
    }
    case 7: {
      using SyncT = message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT,
                                                      MsgT, MsgT, MsgT>;
      this->sync.emplace<SyncT>(*subs[0], *subs[1], *subs[2], *subs[3],
                                *subs[4], *subs[5], *subs[6], qos.depth());
      std::get<SyncT>(this->sync)
          .registerCallback(std::bind(
              &VariableSynchronizer::helper7_cb, this, std::placeholders::_1,
              std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5,
              std::placeholders::_6, std::placeholders::_7));
      break;
    }
    case 8: {
      using SyncT = message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT,
                                                      MsgT, MsgT, MsgT, MsgT>;
      this->sync.emplace<SyncT>(*subs[0], *subs[1], *subs[2], *subs[3],
                                *subs[4], *subs[5], *subs[6], *subs[7],
                                qos.depth());
      std::get<SyncT>(this->sync)
          .registerCallback(
              std::bind(&VariableSynchronizer::helper8_cb, this,
                        std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3, std::placeholders::_4,
                        std::placeholders::_5, std::placeholders::_6,
                        std::placeholders::_7, std::placeholders::_8));
      break;
    }
    case 9: {
      using SyncT =
          message_filters::TimeSynchronizer<MsgT, MsgT, MsgT, MsgT, MsgT, MsgT,
                                            MsgT, MsgT, MsgT>;
      this->sync.emplace<SyncT>(*subs[0], *subs[1], *subs[2], *subs[3],
                                *subs[4], *subs[5], *subs[6], *subs[7],
                                *subs[8], qos.depth());
      std::get<SyncT>(this->sync)
          .registerCallback(std::bind(
              &VariableSynchronizer::helper9_cb, this, std::placeholders::_1,
              std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5,
              std::placeholders::_6, std::placeholders::_7,
              std::placeholders::_8, std::placeholders::_9));
      break;
    }
    default:
      RCLCPP_WARN(node->get_logger(), "Unsupported number of topics");
    }
  }

private:
  rclcpp::Node *node;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
  std::vector<std::unique_ptr<SubT>> subs;
  VariantT sync;
  UserCbT user_cb;

  // This should be possible with lambdas, but isn't
  inline void helper2_cb(const MsgPtrT &msg1, const MsgPtrT &msg2) const {
    this->aggregate_cb(msg1, msg2);
  }
  inline void helper3_cb(const MsgPtrT &msg1, const MsgPtrT &msg2,
                         const MsgPtrT &msg3) const {
    this->aggregate_cb(msg1, msg2, msg3);
  }
  inline void helper4_cb(const MsgPtrT &msg1, const MsgPtrT &msg2,
                         const MsgPtrT &msg3, const MsgPtrT &msg4) const {
    this->aggregate_cb(msg1, msg2, msg3, msg4);
  }
  inline void helper5_cb(const MsgPtrT &msg1, const MsgPtrT &msg2,
                         const MsgPtrT &msg3, const MsgPtrT &msg4,
                         const MsgPtrT &msg5) const {
    this->aggregate_cb(msg1, msg2, msg3, msg4, msg5);
  }
  inline void helper6_cb(const MsgPtrT &msg1, const MsgPtrT &msg2,
                         const MsgPtrT &msg3, const MsgPtrT &msg4,
                         const MsgPtrT &msg5, const MsgPtrT &msg6) const {
    this->aggregate_cb(msg1, msg2, msg3, msg4, msg5, msg6);
  }
  inline void helper7_cb(const MsgPtrT &msg1, const MsgPtrT &msg2,
                         const MsgPtrT &msg3, const MsgPtrT &msg4,
                         const MsgPtrT &msg5, const MsgPtrT &msg6,
                         const MsgPtrT &msg7) const {
    this->aggregate_cb(msg1, msg2, msg3, msg4, msg5, msg6, msg7);
  }
  inline void helper8_cb(const MsgPtrT &msg1, const MsgPtrT &msg2,
                         const MsgPtrT &msg3, const MsgPtrT &msg4,
                         const MsgPtrT &msg5, const MsgPtrT &msg6,
                         const MsgPtrT &msg7, const MsgPtrT &msg8) const {
    this->aggregate_cb(msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8);
  }
  inline void helper9_cb(const MsgPtrT &msg1, const MsgPtrT &msg2,
                         const MsgPtrT &msg3, const MsgPtrT &msg4,
                         const MsgPtrT &msg5, const MsgPtrT &msg6,
                         const MsgPtrT &msg7, const MsgPtrT &msg8,
                         const MsgPtrT &msg9) const {
    this->aggregate_cb(msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8, msg9);
  }

  void aggregate_cb(MsgPtrT msg1 = nullptr, MsgPtrT msg2 = nullptr,
                    MsgPtrT msg3 = nullptr, MsgPtrT msg4 = nullptr,
                    MsgPtrT msg5 = nullptr, MsgPtrT msg6 = nullptr,
                    MsgPtrT msg7 = nullptr, MsgPtrT msg8 = nullptr,
                    MsgPtrT msg9 = nullptr) const {
    std::vector<MsgPtrT> msgs;
    msgs.reserve(9);

    for (auto &msg : {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8, msg9}) {
      if (msg) {
        msgs.push_back(std::move(msg));
      } else {
        break;
      }
    }

    if (user_cb) {
      user_cb(std::move(msgs));
    } else {
      RCLCPP_WARN_ONCE(node->get_logger(),
                       "No callback set for VariableSynchronizer");
    }
  }
};
} // namespace virtcam

#endif // VIRTCAM__SYNCHRONIZER_HPP_
