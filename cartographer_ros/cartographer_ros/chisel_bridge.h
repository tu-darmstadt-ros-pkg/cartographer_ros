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

#ifndef CARTOGRAPHER_ROS_CHISEL_BRIDGE_H_
#define CARTOGRAPHER_ROS_CHISEL_BRIDGE_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

class ChiselBridge {
 public:
  ChiselBridge();
  ChiselBridge(const ChiselBridge&) = delete;
  ChiselBridge& operator=(const ChiselBridge&) = delete;

  void PublishTSDF(MapBuilderBridge* map_builder_bridge);
  void FillMarkerTopicWithMeshes(const chisel::MeshMap& meshMap, visualization_msgs::Marker* marker, int idx = -1);


  ::ros::Publisher mesh_publisher_;
  ::ros::Publisher uncorrected_mesh_publisher_;
  ::ros::Publisher debug_mesh_publisher_;
  ::ros::Publisher normal_publisher_;
  ::ros::Publisher tsdf_pointcloud_publisher_;
  ::ros::Publisher aggregated_scan_publisher_;
  ::ros::Publisher raw_aggregated_scan_publisher_;
  ::ros::Publisher matched_batch_publisher_;
  ::ros::Publisher volume_publisher_;
  ::ros::Publisher incremental_changes_publisher_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CHISEL_BRIDGE_H_
