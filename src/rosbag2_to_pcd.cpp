// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 M. Fatih Cırıt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/rosbag2_to_pcd.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <exception>
#include <filesystem>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

std::string getTimestampString(
  const sensor_msgs::msg::PointCloud2 & msg_cloud,
  const int max_sec)
{
  std::stringstream ss_timestamp;

  // Seconds, zero-padded to width of max_sec
  if (0 <= max_sec && msg_cloud.header.stamp.sec < max_sec) {
    const int sec_width = std::to_string(max_sec).length();
    ss_timestamp << std::setw(sec_width) << std::setfill('0');
  }
  ss_timestamp << msg_cloud.header.stamp.sec;

  // Separator
  ss_timestamp << "-";

  // Nanoseconds, always zero-padded to 9 digits
  ss_timestamp << std::setw(9) << std::setfill('0') << msg_cloud.header.stamp.nanosec;

  return ss_timestamp.str();
}

}  // Anonymous namespace

namespace rosbag2_to_pcd
{
Rosbag2ToPcdNode::Rosbag2ToPcdNode(const rclcpp::NodeOptions & node_options)
: Node("rosbag2_to_pcd", node_options)
{
  RCLCPP_INFO(get_logger(), "Starting rosbag2_to_pcd node...");
  const std::string path_bag = this->declare_parameter<std::string>("path_bag");
  const std::string topic_cloud = this->declare_parameter<std::string>("topic_cloud");

  const std::string path_pcds = path_bag + "_pcds/";

  std::filesystem::create_directory(path_pcds);

  rosbag2_cpp::Reader reader;
  try {
    reader.open(path_bag);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error opening bag file: " << e.what());
    rclcpp::shutdown();
    return;
  }

  // First pass to find max seconds for timestamp formatting.
  int max_sec = 0;
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;
    auto msg_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

    rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    serialization.deserialize_message(&extracted_serialized_msg, &(*msg_cloud));

    if (msg_cloud->header.stamp.sec > max_sec) {
      max_sec = msg_cloud->header.stamp.sec;
    }
  }

  reader.seek(0);

  // Second pass to convert and save point clouds.
  const auto & topics = reader.get_metadata().topics_with_message_count;
  const auto iter_topic =
    std::find_if(topics.begin(), topics.end(), [&topic_cloud](const auto & topic) {
      return topic.topic_metadata.name == topic_cloud;
    });

  if (iter_topic == topics.end()) {
    throw std::runtime_error("Topic not found in the bag file.");
  }

  const size_t message_count = iter_topic->message_count;
  int ctr_msg_cloud = 1;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

  while (reader.has_next()) {
    auto bag_message = reader.read_next();

    if (bag_message->topic_name == topic_cloud) {
      using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;
      auto msg_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization.deserialize_message(&extracted_serialized_msg, &(*msg_cloud));

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*msg_cloud, *cloud);

      const std::string timestamp = getTimestampString(*msg_cloud, max_sec);

      RCLCPP_INFO_STREAM(
        get_logger(),
        "timestamp: " << timestamp << " #" << ctr_msg_cloud << " of " << message_count);

      std::string filename = path_pcds + timestamp + ".pcd";

      pcl::io::savePCDFileASCII(filename, *cloud);
      ctr_msg_cloud++;
    }
  }

  RCLCPP_INFO(get_logger(), "Finished converting bag file to pcd files.");
  rclcpp::shutdown();
}

}  // namespace rosbag2_to_pcd

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rosbag2_to_pcd::Rosbag2ToPcdNode)
