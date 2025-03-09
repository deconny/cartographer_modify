/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros_msgs/srv/switch_mode.hpp"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(mapping_configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(localization_configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");              
// DEFINE_string(load_state_filename, "",
//               "If non-empty, filename of a .pbstream file to load, containing "
//               "a saved SLAM state.");

DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "test.pbstream",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {
class CartographerNode : public rclcpp::Node {
 public:
  CartographerNode();
  ~CartographerNode();
  void init();
  void stopNode();

 private:
  void switchModeCallback(
      const std::shared_ptr<cartographer_ros_msgs::srv::SwitchMode::Request>
          request,
      std::shared_ptr<cartographer_ros_msgs::srv::SwitchMode::Response>
          response);
  void startMappingMode();
  bool startLocalizationMode(const std::string& map_filename);
  void createNode(const std::string& mode);
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  double kTfBufferCacheTimeInSeconds_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  NodeOptions node_options_;
  TrajectoryOptions trajectory_options_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::SwitchMode>::SharedPtr
      switch_mode_server_;
  std::string current_mode_;
  std::string mapping_configuration_basename_;
  std::string localization_configuration_basename_;
  std::shared_ptr<cartographer_ros::Cartographer> node_;
  std::mutex mutex_;  // mutex for node_
};

CartographerNode::CartographerNode()
    : Node("cartographer_ros_node"),
      kTfBufferCacheTimeInSeconds_(10.),
      current_mode_("mapping") {}

CartographerNode::~CartographerNode() {
  std::lock_guard<std::mutex> lock(mutex_);
  stopNode();
}

void CartographerNode::init() {
  std::lock_guard<std::mutex> lock(mutex_);
  mapping_configuration_basename_ = FLAGS_mapping_configuration_basename;
  localization_configuration_basename_ = FLAGS_localization_configuration_basename;
  // localization_configuration_basename_ = FLAGS_configuration_basename;
  createNode(current_mode_);
  if (FLAGS_start_trajectory_with_default_topics) {
    node_->StartTrajectoryWithDefaultTopics(trajectory_options_);
  }    
  switch_mode_server_ =
      this->create_service<cartographer_ros_msgs::srv::SwitchMode>(
          "switch_mode",
          std::bind(&CartographerNode::switchModeCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
}

void CartographerNode::createNode(const std::string& mode) {
  if (mode == "mapping") {
    std::tie(node_options_, trajectory_options_) = LoadOptions(
        FLAGS_configuration_directory, mapping_configuration_basename_);
  } else if (mode == "localization") {
    std::tie(node_options_, trajectory_options_) = LoadOptions(
        FLAGS_configuration_directory, localization_configuration_basename_);
  }
  auto map_builder =
      cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
          node_options_.map_builder_options);
  node_ = std::make_shared<cartographer_ros::Cartographer>(
      node_options_, this->shared_from_this(), std::move(map_builder));
}

void CartographerNode::stopNode() {
  if (node_) {
    node_->FinishAllTrajectories();
    node_->RunFinalOptimization();
    node_.reset();
  }
}

void CartographerNode::startMappingMode() {
  // Start mapping mode
  RCLCPP_INFO(this->get_logger(), "Switching to Mapping mode.");
  stopNode();
  createNode("mapping");
  if (FLAGS_start_trajectory_with_default_topics) {
    node_->StartTrajectoryWithDefaultTopics(trajectory_options_);
  }  
}

bool CartographerNode::startLocalizationMode(const std::string& map_filename) {
  // Start localization mode
  if (map_filename.empty() || map_filename.find(".pbstream") == std::string::npos) {
    RCLCPP_ERROR(this->get_logger(), "Map filename is empty");
    return false;
  }
  stopNode();
  createNode("localization");
  try {
    node_->LoadState(map_filename, FLAGS_load_frozen_state);
    RCLCPP_INFO(this->get_logger(), "Successfully loaded state from '%s'.",
                map_filename.c_str());
  if (FLAGS_start_trajectory_with_default_topics) {
    node_->StartTrajectoryWithDefaultTopics(trajectory_options_);
  }                
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while loading state: %s",
                 e.what());
    return false;
  }
  return true;
}

void CartographerNode::switchModeCallback(
    const std::shared_ptr<cartographer_ros_msgs::srv::SwitchMode::Request>
        request,
    std::shared_ptr<cartographer_ros_msgs::srv::SwitchMode::Response>
        response) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (request->mode != current_mode_) {
    if (request->mode == "mapping") {
      startMappingMode();
      current_mode_ = request->mode;
      response->success = true;
      response->message = "Switched to Mapping mode.";
    } else if (request->mode == "localization") {
      if (!startLocalizationMode(request->map_filename)) {
        response->success = false;
        response->message = "Failed to start localization mode.";
        return;
      }
      current_mode_ = request->mode;
      response->success = true;
      response->message = "Switched to Localization mode.";
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid mode: %s",
                   request->mode.c_str());
      response->success = false;
      response->message = "Invalid mode: " + request->mode;
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Already in '%s' mode. No action taken.",
                current_mode_.c_str());
    response->success = true;
    response->message = "Already in the requested mode.";
  }
}
}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_mapping_configuration_basename.empty())
      << "-configuration_basename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  auto cartographer_ros_node =
      std::make_shared<cartographer_ros::CartographerNode>();
  cartographer_ros_node->init();
  ::rclcpp::spin(cartographer_ros_node);
  ::rclcpp::shutdown();
}