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
#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <queue>
#include <string>
#include <vector>

// tf2
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

// rosbag headers
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

class BufferableBag {
public:
    // Wrapper node to process the transforamtions present in the bagfile
    struct TFBridge {
        TFBridge(rclcpp::Node::SharedPtr node);
        void ProcessTFMessage(std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) const;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
        rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
    };

    BufferableBag(const std::string &bag_filename,
                  const std::shared_ptr<TFBridge> tf_bridge,
                  const std::string &topic,
                  const std::chrono::seconds buffer_size = std::chrono::seconds(1));

    std::size_t message_count() const;
    void BufferMessages();
    rosbag2_storage::SerializedBagMessage PopNextMessage();
    bool finished() const;

private:
    std::shared_ptr<TFBridge> tf_bridge_;
    std::unique_ptr<rosbag2_cpp::Reader> bag_reader_;
    std::queue<rosbag2_storage::SerializedBagMessage> buffer_;
    std::chrono::seconds buffer_size_;
    std::string topic_;
    std::size_t message_count_{0};
};

class BagMultiplexer {
public:
    // assume we feed this bags in ordered fashion, plesae behave
    void AddBag(BufferableBag &&bag);

    // How many messages in total in all the bagfiles
    std::size_t message_count() const;

    rosbag2_storage::SerializedBagMessage GetNextMessage();
    bool IsMessageAvailable() const;

private:
    std::vector<BufferableBag> bags_;
    std::size_t current_index_{0};
    std::size_t message_count_{0};
};
