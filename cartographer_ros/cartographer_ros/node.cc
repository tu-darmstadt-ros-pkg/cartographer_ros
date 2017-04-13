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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_eigen/tf2_eigen.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

constexpr int kLatestOnlyPublisherQueueSize = 1;

Node::Node(const NodeOptions& options, tf2_ros::Buffer* const tf_buffer)
    : options_(options), map_builder_bridge_(options_, tf_buffer) {}

Node::~Node() {
  {
    carto::common::MutexLocker lock(&mutex_);
    terminating_ = true;
  }
  if (occupancy_grid_thread_.joinable()) {
    occupancy_grid_thread_.join();
  }
}

void Node::Initialize() {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);

  mesh_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("chisel_mesh", 1);
  //normal_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("chisel_normals", 1);
  tsdf_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("chisel_tsdf", 1);
  submap_query_server_ = node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this);

  if (options_.map_builder_options.use_trajectory_builder_2d()) {
    occupancy_grid_publisher_ =
        node_handle_.advertise<::nav_msgs::OccupancyGrid>(
            kOccupancyGridTopic, kLatestOnlyPublisherQueueSize,
            true /* latched */);
    occupancy_grid_thread_ =
        std::thread(&Node::SpinOccupancyGridThreadForever, this);
  }

  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));  
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(options_.submap_publish_period_sec*10.0),
      &Node::PublishTSDF, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(options_.pose_publish_period_sec),
      &Node::PublishTrajectoryStates, this));
}

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

MapBuilderBridge* Node::map_builder_bridge() { return &map_builder_bridge_; }

bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  return map_builder_bridge_.HandleSubmapQuery(request, response);
}

void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {

  if(submap_list_publisher_.getNumSubscribers() > 0){
      carto::common::MutexLocker lock(&mutex_);
      submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
  }
}

void Node::PublishTSDF(const ::ros::WallTimerEvent& unused_timer_event) {
    std::vector<chisel::ChiselPtr> tsdf_list = map_builder_bridge_.GetTSDFList();
    if(tsdf_list.size() > 0 && tsdf_publisher_.getNumSubscribers() > 0){
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.clear();
        for(const chisel::ChiselPtr chisel_map : tsdf_list)
        {
            if(chisel_map)
            {
                const chisel::ChunkManager& chunkManager = chisel_map->GetChunkManager();
                const float resolution = chunkManager.GetResolution();
                chisel::Vec3 map_offset = chunkManager.GetMapOffset();
                int stepSize=1;

                for (const std::pair<chisel::ChunkID, chisel::ChunkPtr>& pair : chunkManager.GetChunks())
                {
                  const std::vector<chisel::DistVoxel>&  voxels = pair.second->GetVoxels();
                  chisel::Vec3 origin = pair.second->GetOrigin();

                  int voxelID = 0;

                  for (int z = 0; z < chunkManager.GetChunkSize()(2); z+=stepSize)
                  {
                    for (int y = 0; y < chunkManager.GetChunkSize()(1); y+=stepSize)
                    {
                      for (int x = 0; x < chunkManager.GetChunkSize()(0); x+=stepSize)
                      {
                        if(voxels[voxelID].GetWeight() > 0)
                        {

                            float sdf = voxels[voxelID].GetSDF();

                            if(sdf>0)
                            {
                              pcl::PointXYZRGB point = pcl::PointXYZRGB(0, 0, 255);
                              point.x = map_offset.x() + origin.x() + x *resolution;
                              point.y = map_offset.y() + origin.y() + y *resolution;
                              point.z = map_offset.z() + origin.z() + z *resolution;
                              cloud.points.insert(cloud.end(), point);
                            }
                            else
                            {
                              pcl::PointXYZRGB point = pcl::PointXYZRGB(255, 0, 0);
                              point.x = map_offset.x() + origin.x() + x *resolution;
                              point.y = map_offset.y() + origin.y() + y *resolution;
                              point.z = map_offset.z() + origin.z() + z *resolution;
                              cloud.points.insert(cloud.end(), point);
                            }
                        }

                        voxelID+=stepSize;
                      }
                    }
                  }
                }
            }
        }
        sensor_msgs::PointCloud2 pc;
        pcl::toROSMsg(cloud, pc);
        pc.header.frame_id = "map";
        pc.header.stamp = ros::Time::now();
        tsdf_publisher_.publish(pc);
    }


    if(tsdf_list.size() > 0 && mesh_publisher_.getNumSubscribers() > 0){
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.reserve(tsdf_list.size());
        int id = 0;
        for(const chisel::ChiselPtr chisel_map : tsdf_list)
        {
            if(chisel_map)
            {
                const chisel::ChunkManager& chunkManager = chisel_map->GetChunkManager();
                chisel::Vec3 map_offset = chunkManager.GetMapOffset();
                visualization_msgs::Marker marker;
                marker.header.stamp = ros::Time::now();
                marker.header.frame_id = "map";
                marker.id = id;
                marker.scale.x = 1;
                marker.scale.y = 1;
                marker.scale.z = 1;
                marker.pose.orientation.x = 0;
                marker.pose.orientation.y = 0;
                marker.pose.orientation.z = 0;
                marker.pose.orientation.w = 1;
                marker.pose.position.x = map_offset.x();
                marker.pose.position.y = map_offset.y();
                marker.pose.position.z = map_offset.z();
                marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
                const chisel::MeshMap& mesh_map = chisel_map->GetChunkManager().GetAllMeshes();
                FillMarkerTopicWithMeshes(mesh_map, &marker);
                if(marker.points.size() > 0)
                {
                    marker_array.markers.push_back(marker);
                    id++;
                }
            }
        }

        mesh_publisher_.publish(marker_array);
    }

}

chisel::Vec3 LAMBERT(const chisel::Vec3& n, const chisel::Vec3& light)
{
    return fmax(n.dot(light), 0.0f) * chisel::Vec3(0.5, 0.5, 0.5);
}

void Node::FillMarkerTopicWithMeshes(const chisel::MeshMap& meshMap, visualization_msgs::Marker* marker)
{
    if(meshMap.size() == 0)
    {
        return;
    }

    chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
    lightDir.normalize();
    chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
    lightDir.normalize();
    const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);
    //int idx = 0;
    for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
    {
        const chisel::MeshPtr& mesh = meshes.second;
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const chisel::Vec3& vec = mesh->vertices[i];
            geometry_msgs::Point pt;
            pt.x = vec[0];
            pt.y = vec[1];
            pt.z = vec[2];
            marker->points.push_back(pt);

            if(mesh->HasColors())
            {
                const chisel::Vec3& meshCol = mesh->colors[i];
                std_msgs::ColorRGBA color;
                color.r = meshCol[0];
                color.g = meshCol[1];
                color.b = meshCol[2];
                color.a = 1.0;
                marker->colors.push_back(color);
            }
            else
            {
              if(mesh->HasNormals())
              {
                  const chisel::Vec3 normal = mesh->normals[i];
                  std_msgs::ColorRGBA color;
                  chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
                  color.r = fmin(lambert[0], 1.0);
                  color.g = fmin(lambert[1], 1.0);
                  color.b = fmin(lambert[2], 1.0);
                  color.a = 1.0;
                  marker->colors.push_back(color);
              }
              else
              {
                std_msgs::ColorRGBA color;
                color.r = vec[0] * 0.25 + 0.5;
                color.g = vec[1] * 0.25 + 0.5;
                color.b = vec[2] * 0.25 + 0.5;
                color.a = 1.0;
                marker->colors.push_back(color);
              }
            }
            //marker->indicies.push_back(idx);
            //idx++;
        }
    }
}

void Node::PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    const auto& trajectory_state = entry.second;

    geometry_msgs::TransformStamped stamped_transform;
    stamped_transform.header.stamp = ToRos(trajectory_state.pose_estimate.time);

    const auto& tracking_to_local = trajectory_state.pose_estimate.pose;
    const Rigid3d tracking_to_map =
        trajectory_state.local_to_map * tracking_to_local;

    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_state.pose_estimate.time !=
        last_scan_matched_point_cloud_time_) {
      scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_state.pose_estimate.time),
          options_.tracking_frame,
          carto::sensor::TransformPointCloud(
              trajectory_state.pose_estimate.point_cloud,
              tracking_to_local.inverse().cast<float>())));
      last_scan_matched_point_cloud_time_ = trajectory_state.pose_estimate.time;
    } else {
      // If we do not publish a new point cloud, we still allow time of the
      // published poses to advance.
      stamped_transform.header.stamp = ros::Time::now();
    }

    if (trajectory_state.published_to_tracking != nullptr) {
      if (options_.provide_odom_frame) {
        std::vector<geometry_msgs::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = options_.map_frame;
        // TODO(damonkohler): 'odom_frame' and 'published_frame' must be
        // per-trajectory to fully support the multi-robot use case.
        stamped_transform.child_frame_id = options_.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_state.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id = options_.odom_frame;
        stamped_transform.child_frame_id = options_.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_state.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_.sendTransform(stamped_transforms);
      } else {
        stamped_transform.header.frame_id = options_.map_frame;
        stamped_transform.child_frame_id = options_.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_map * (*trajectory_state.published_to_tracking));
        tf_broadcaster_.sendTransform(stamped_transform);
      }
    }
  }
}

void Node::SpinOccupancyGridThreadForever() {
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    {
      carto::common::MutexLocker lock(&mutex_);
      if (terminating_) {
        return;
      }
    }
    if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
      continue;
    }
    const auto occupancy_grid = map_builder_bridge_.BuildOccupancyGrid();
    if (occupancy_grid != nullptr) {
      occupancy_grid_publisher_.publish(*occupancy_grid);
    }
  }
}

}  // namespace cartographer_ros
