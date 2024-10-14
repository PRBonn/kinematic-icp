// MIT License

// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "kinematic_icp_ros/utils/RosbagUtils.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

// ROS
#include <rclcpp/version.h>

#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace {
auto GetTimestampsFromRosbagSerializedMsg(const rosbag2_storage::SerializedBagMessage &msg) {
#if RCLCPP_VERSION_GTE(22, 0, 0)
    return std::chrono::nanoseconds(msg.send_timestamp);
#else
    return std::chrono::nanoseconds(msg.time_stamp);
#endif
}
}  // namespace

/// TFBridge----------------------------------------------------------------------------------------
BufferableBag::TFBridge::TFBridge(rclcpp::Node::SharedPtr node) {
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    tf_static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
}

void BufferableBag::TFBridge::ProcessTFMessage(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) const {
    tf2_msgs::msg::TFMessage tf_message;
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    serializer.deserialize_message(&serialized_msg, &tf_message);
    // Broadcast tranforms to /tf and /tf_static topics
    for (auto &transform : tf_message.transforms) {
        if (msg->topic_name == "/tf_static") {
            tf_static_broadcaster->sendTransform(transform);
        } else {
            tf_broadcaster->sendTransform(transform);
        }
    }
}

/// BufferableBag-----------------------------------------------------------------------------------
BufferableBag::BufferableBag(const std::string &bag_filename,
                             const std::shared_ptr<TFBridge> tf_bridge,
                             const std::string &topic,
                             const std::chrono::seconds buffer_size)
    : tf_bridge_(tf_bridge),
      bag_reader_(std::make_unique<rosbag2_cpp::Reader>()),
      topic_(topic),
      buffer_size_(buffer_size) {
    bag_reader_->open(bag_filename);
    message_count_ = [&]() {
        std::size_t message_count = 0;
        const auto &metadata = bag_reader_->get_metadata();
        const auto topic_info = metadata.topics_with_message_count;
        const auto it = std::find_if(topic_info.begin(), topic_info.end(), [&](const auto &info) {
            return info.topic_metadata.name == topic_;
        });
        if (it != topic_info.end()) {
            message_count += it->message_count;
        }
        return message_count;
    }();

    BufferMessages();
}

bool BufferableBag::finished() const { return !bag_reader_->has_next(); };

std::size_t BufferableBag::message_count() const { return message_count_; }

void BufferableBag::BufferMessages() {
    auto buffer_is_filled = [&]() -> bool {
        if (buffer_.empty()) return false;
        const auto first_stamp = GetTimestampsFromRosbagSerializedMsg(buffer_.front());
        const auto last_stamp = GetTimestampsFromRosbagSerializedMsg(buffer_.back());
        return (last_stamp - first_stamp) > buffer_size_;
    };

    // Advance reading one message until the buffer is filled or we finish the bagfile
    while (!finished() && !buffer_is_filled()) {
        // Fetch next message from bagfile, could be anything
        const auto msg = bag_reader_->read_next();
        // TODO(Nacho): The following logic should be customizable from the outside world
        // If the msg is TFMessage, fill the tf_buffer and broadcast the transformation and don't
        // populate the buffered_messages_ as we already processed it
        if (msg->topic_name == "/tf" || msg->topic_name == "/tf_static") {
            tf_bridge_->ProcessTFMessage(msg);
        } else if (msg->topic_name == topic_) {
            // If the msg is not TFMessage then push it to the interal buffer of all messages
            buffer_.push(*msg);
        }
    }
}

rosbag2_storage::SerializedBagMessage BufferableBag::PopNextMessage() {
    const rosbag2_storage::SerializedBagMessage msg = buffer_.front();
    buffer_.pop();
    BufferMessages();
    return msg;
}

/// Multiplexer ------------------------------------------------------------------------------------
void BagMultiplexer::AddBag(BufferableBag &&bag) {
    message_count_ += bag.message_count();
    bags_.push_back(std::move(bag));
}

std::size_t BagMultiplexer::message_count() const { return message_count_; };

bool BagMultiplexer::IsMessageAvailable() const { return !bags_[current_index_].finished(); }

rosbag2_storage::SerializedBagMessage BagMultiplexer::GetNextMessage() {
    if (bags_[current_index_].finished() && current_index_ + 1 < bags_.size()) {
        current_index_++;
    }
    return bags_[current_index_].PopNextMessage();
}
