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
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
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

namespace {

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

// Try to convert 'msg' into 'options'. Returns false on failure.
bool FromRosMessage(const cartographer_ros_msgs::TrajectoryOptions& msg,
                    TrajectoryOptions* options) {
  options->tracking_frame = msg.tracking_frame;
  options->published_frame = msg.published_frame;
  options->odom_frame = msg.odom_frame;
  options->provide_odom_frame = msg.provide_odom_frame;
  options->use_odometry = msg.use_odometry;
  options->use_laser_scan = msg.use_laser_scan;
  options->use_multi_echo_laser_scan = msg.use_multi_echo_laser_scan;
  options->num_point_clouds = msg.num_point_clouds;
  if (!options->trajectory_builder_options.ParseFromString(
          msg.trajectory_builder_options_proto)) {
    LOG(ERROR) << "Failed to parse protobuf";
    return false;
  }
  return true;
}

void ShutdownSubscriber(std::unordered_map<int, ::ros::Subscriber>& subscribers,
                        int trajectory_id) {
  if (subscribers.count(trajectory_id) == 0) {
    return;
  }
  subscribers[trajectory_id].shutdown();
  LOG(INFO) << "Shutdown the subscriber of ["
            << subscribers[trajectory_id].getTopic() << "]";
  CHECK_EQ(subscribers.erase(trajectory_id), 1);
}

bool IsTopicNameUnique(
    const string& topic,
    const std::unordered_map<int, ::ros::Subscriber>& subscribers) {
  for (auto& entry : subscribers) {
    if (entry.second.getTopic() == topic) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

}  // namespace

Node::Node(const NodeOptions& node_options, tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, tf_buffer) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteAssetsServiceName, &Node::HandleWriteAssets, this));

  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    occupancy_grid_publisher_ =
        node_handle_.advertise<::nav_msgs::OccupancyGrid>(
            kOccupancyGridTopic, kLatestOnlyPublisherQueueSize,
            true /* latched */);
    occupancy_grid_thread_ =
        std::thread(&Node::SpinOccupancyGridThreadForever, this);
  }

  mesh_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("chisel_mesh", 1);
  uncorrected_mesh_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("uncorrected_chisel_mesh", 1);
  debug_mesh_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("debug_chisel_mesh", 1);
  tsdf_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("chisel_tsdf", 1);
  aggregated_scan_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("aggregated_scan", 1);
  raw_aggregated_scan_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("raw_aggregated_scan", 1);
  matched_batch_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("matched_batch", 1);

  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.pose_publish_period_sec),
      &Node::PublishTrajectoryStates, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec * 5.0),
      &Node::PublishTSDF, this));
}

Node::~Node() {
  {
    carto::common::MutexLocker lock(&mutex_);
    terminating_ = true;
  }
  if (occupancy_grid_thread_.joinable()) {
    occupancy_grid_thread_.join();
  }
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
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
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
          trajectory_state.trajectory_options.tracking_frame,
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
      if (trajectory_state.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_state.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_state.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_.sendTransform(stamped_transforms);
      } else {
        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
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

int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::SensorTopics& topics) {
  std::unordered_set<string> expected_sensor_ids;

  if (options.use_laser_scan) {
    expected_sensor_ids.insert(topics.laser_scan_topic);
  }
  if (options.use_multi_echo_laser_scan) {
    expected_sensor_ids.insert(topics.multi_echo_laser_scan_topic);
  }
  if (options.num_point_clouds > 0) {
    for (int i = 0; i < options.num_point_clouds; ++i) {
      string topic = topics.point_cloud2_topic;
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      expected_sensor_ids.insert(topic);
    }
  }
  if (options.trajectory_builder_options.trajectory_builder_2d_options()
          .use_imu_data()) {
    expected_sensor_ids.insert(topics.imu_topic);
  }
  if (options.use_odometry) {
    expected_sensor_ids.insert(topics.odometry_topic);
  }
  return map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::SensorTopics& topics,
                             const int trajectory_id) {
  if (options.use_laser_scan) {
    const string topic = topics.laser_scan_topic;
    laser_scan_subscribers_[trajectory_id] =
        node_handle_.subscribe<sensor_msgs::LaserScan>(
            topic, kInfiniteSubscriberQueueSize,
            boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
                [this, trajectory_id,
                 topic](const sensor_msgs::LaserScan::ConstPtr& msg) {
                  map_builder_bridge_.sensor_bridge(trajectory_id)
                      ->HandleLaserScanMessage(topic, msg);
                }));
  }

  if (options.use_multi_echo_laser_scan) {
    const string topic = topics.multi_echo_laser_scan_topic;
    multi_echo_laser_scan_subscribers_[trajectory_id] =
        node_handle_.subscribe<sensor_msgs::MultiEchoLaserScan>(
            topic, kInfiniteSubscriberQueueSize,
            boost::function<void(
                const sensor_msgs::MultiEchoLaserScan::ConstPtr&)>(
                [this, trajectory_id,
                 topic](const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
                  map_builder_bridge_.sensor_bridge(trajectory_id)
                      ->HandleMultiEchoLaserScanMessage(topic, msg);
                }));
  }

  std::vector<::ros::Subscriber> grouped_point_cloud_subscribers;
  if (options.num_point_clouds > 0) {
    for (int i = 0; i < options.num_point_clouds; ++i) {
      string topic = topics.point_cloud2_topic;
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      grouped_point_cloud_subscribers.push_back(node_handle_.subscribe(
          topic, kInfiniteSubscriberQueueSize,
          boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>(
              [this, trajectory_id,
               topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                map_builder_bridge_.sensor_bridge(trajectory_id)
                    ->HandlePointCloud2Message(topic, msg);
              })));
    }
    point_cloud_subscribers_[trajectory_id] = grouped_point_cloud_subscribers;
  }

  if (options.trajectory_builder_options.trajectory_builder_2d_options()
          .use_imu_data()) {
    string topic = topics.imu_topic;
    imu_subscribers_[trajectory_id] = node_handle_.subscribe<sensor_msgs::Imu>(
        topic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::Imu::ConstPtr&)>(
            [this, trajectory_id,
             topic](const sensor_msgs::Imu::ConstPtr& msg) {
              map_builder_bridge_.sensor_bridge(trajectory_id)
                  ->HandleImuMessage(topic, msg);
            }));
  }

  if (options.use_odometry) {
    string topic = topics.odometry_topic;
    odom_subscribers_[trajectory_id] =
        node_handle_.subscribe<nav_msgs::Odometry>(
            topic, kInfiniteSubscriberQueueSize,
            boost::function<void(const nav_msgs::Odometry::ConstPtr&)>(
                [this, trajectory_id,
                 topic](const nav_msgs::Odometry::ConstPtr& msg) {
                  map_builder_bridge_.sensor_bridge(trajectory_id)
                      ->HandleOdometryMessage(topic, msg);
                }));
  }

  is_active_trajectory_[trajectory_id] = true;
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d() &&
      options.trajectory_builder_options.has_trajectory_builder_2d_options()) {
    // Using point clouds is only supported in 3D.
    if (options.num_point_clouds == 0) {
      return true;
    }
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d() &&
      options.trajectory_builder_options.has_trajectory_builder_3d_options()) {
    if (options.num_point_clouds != 0) {
      return true;
    }
  }
  return false;
}

bool Node::ValidateTopicName(
    const ::cartographer_ros_msgs::SensorTopics& topics,
    const TrajectoryOptions& options) {
  if (!IsTopicNameUnique(topics.laser_scan_topic, laser_scan_subscribers_)) {
    return false;
  }
  if (!IsTopicNameUnique(topics.multi_echo_laser_scan_topic,
                         multi_echo_laser_scan_subscribers_)) {
    return false;
  }
  if (!IsTopicNameUnique(topics.imu_topic, imu_subscribers_)) {
    return false;
  }
  if (!IsTopicNameUnique(topics.odometry_topic, odom_subscribers_)) {
    return false;
  }
  for (auto& subscribers : point_cloud_subscribers_) {
    string topic = topics.point_cloud2_topic;
    int count = 0;
    for (auto& subscriber : subscribers.second) {
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(count + 1);
        ++count;
      }
      if (subscriber.getTopic() == topic) {
        LOG(ERROR) << "Topic name [" << topic << "] is already used";
        return false;
      }
    }
  }
  return true;
}

bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  if (!FromRosMessage(request.options, &options) ||
      !Node::ValidateTrajectoryOptions(options)) {
    LOG(ERROR) << "Invalid trajectory options.";
    return false;
  }
  if (!Node::ValidateTopicName(request.topics, options)) {
    LOG(ERROR) << "Invalid topics.";
    return false;
  }

  std::unordered_set<string> expected_sensor_ids;
  const int trajectory_id = AddTrajectory(options, request.topics);
  LaunchSubscribers(options, request.topics, trajectory_id);

  is_active_trajectory_[trajectory_id] = true;
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  cartographer_ros_msgs::SensorTopics topics;
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;

  const int trajectory_id = AddTrajectory(options, topics);
  LaunchSubscribers(options, topics, trajectory_id);
}

bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id = request.trajectory_id;
  if (is_active_trajectory_.count(trajectory_id) == 0) {
    LOG(INFO) << "Trajectory_id " << trajectory_id << " is not created yet.";
    return false;
  }
  if (!is_active_trajectory_[trajectory_id]) {
    LOG(INFO) << "Trajectory_id " << trajectory_id
              << " has already been finished.";
    return false;
  }

  ShutdownSubscriber(laser_scan_subscribers_, trajectory_id);
  ShutdownSubscriber(multi_echo_laser_scan_subscribers_, trajectory_id);
  ShutdownSubscriber(odom_subscribers_, trajectory_id);
  ShutdownSubscriber(imu_subscribers_, trajectory_id);

  if (point_cloud_subscribers_.count(trajectory_id) != 0) {
    for (auto& entry : point_cloud_subscribers_[trajectory_id]) {
      LOG(INFO) << "Shutdown the subscriber of [" << entry.getTopic() << "]";
      entry.shutdown();
    }
    CHECK_EQ(point_cloud_subscribers_.erase(trajectory_id), 1);
  }
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
  return true;
}

bool Node::HandleWriteAssets(
    ::cartographer_ros_msgs::WriteAssets::Request& request,
    ::cartographer_ros_msgs::WriteAssets::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.WriteAssets(request.stem);
  return true;
}

void Node::FinishAllTrajectories() {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : is_active_trajectory_) {
    const int trajectory_id = entry.first;
    if (entry.second) {
      map_builder_bridge_.FinishTrajectory(trajectory_id);
    }
  }
}

void Node::PublishTSDF(const ::ros::WallTimerEvent& unused_timer_event) {

    std::vector<chisel::ChiselPtr<chisel::DistVoxel>> tsdf_list = map_builder_bridge_.GetTSDFList();
    int trajectory_id = 0;
    const cartographer::mapping::Submaps* const submaps =
            map_builder_bridge()->map_builder_.GetTrajectoryBuilder(trajectory_id)->submaps();

    const std::vector<cartographer::transform::Rigid3d> submap_transforms =
        map_builder_bridge()->map_builder_.sparse_pose_graph()->GetSubmapTransforms(trajectory_id);

    if(tsdf_list.size() > 0 && tsdf_publisher_.getNumSubscribers() > 0){
        //todo(kdaun) add global transform for loop closure correction
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.clear();
        int submap_index = 0;
        for(const chisel::ChiselPtr<chisel::DistVoxel> chisel_map : tsdf_list)
        {
            if (submap_index < 0 || submap_index >= submaps->size()) {
              ROS_INFO( "Requested submap %i from trajectory %i but there are only %i submaps in this trajectory."
                        ,submap_index, trajectory_id, submaps->size());
            }
            if(chisel_map)
            {
                const auto& chunkManager = chisel_map->GetChunkManager();
                const float resolution = chunkManager.GetResolution();
                chisel::Vec3 map_offset = chunkManager.GetOrigin();

                int stepSize=1;

                for (const std::pair<chisel::ChunkID, chisel::ChunkPtr<chisel::DistVoxel>>& pair : chunkManager.GetChunks())
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
            submap_index++;
        }
        sensor_msgs::PointCloud2 pc;
        pcl::toROSMsg(cloud, pc);
        pc.header.frame_id = "map";
        pc.header.stamp = ros::Time::now();
        tsdf_publisher_.publish(pc);
    }


    if(tsdf_list.size() > 0 && debug_mesh_publisher_.getNumSubscribers() > 0){
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.reserve(tsdf_list.size());
        int id = 0;
        int submap_index = 0;
        for(const chisel::ChiselPtr<chisel::DistVoxel> chisel_map : tsdf_list)
        {
            if(chisel_map)
            {
                const auto& chunkManager = chisel_map->GetChunkManager();
                chisel::Vec3 map_offset = chunkManager.GetOrigin();
                visualization_msgs::Marker marker;
                marker.header.stamp = ros::Time::now();
                marker.header.frame_id = "map";
                marker.id = id;
                marker.scale.x = 1;
                marker.scale.y = 1;
                marker.scale.z = 1;
                marker.pose.orientation.x = submap_transforms[submap_index].rotation().x();
                marker.pose.orientation.y = submap_transforms[submap_index].rotation().y();
                marker.pose.orientation.z = submap_transforms[submap_index].rotation().z();
                marker.pose.orientation.w = submap_transforms[submap_index].rotation().w();
                marker.pose.position.x = submap_transforms[submap_index].translation().x();
                marker.pose.position.y = submap_transforms[submap_index].translation().y();
                marker.pose.position.z = submap_transforms[submap_index].translation().z();
                marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
                const chisel::MeshMap& mesh_map = chisel_map->GetChunkManager().GetAllMeshes();
                FillMarkerTopicWithMeshes(mesh_map, &marker, id);
                if(marker.points.size() > 0)
                {
                    marker_array.markers.push_back(marker);
                    id++;
                }
            }
            submap_index++;
        }
        debug_mesh_publisher_.publish(marker_array);
    }

    if(tsdf_list.size() > 0 && uncorrected_mesh_publisher_.getNumSubscribers() > 0){
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.reserve(tsdf_list.size());
        int id = tsdf_list.size();
        for(const chisel::ChiselPtr<chisel::DistVoxel> chisel_map : tsdf_list)
        {
            if(chisel_map)
            {
                const auto& chunkManager = chisel_map->GetChunkManager();
                chisel::Vec3 map_offset = chunkManager.GetOrigin();
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

        uncorrected_mesh_publisher_.publish(marker_array);
    }

    if(tsdf_list.size() > 0 && mesh_publisher_.getNumSubscribers() > 0){
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.reserve(tsdf_list.size());
        int id = 2*tsdf_list.size();
        int submap_index = 0;
        for(const chisel::ChiselPtr<chisel::DistVoxel> chisel_map : tsdf_list)
        {
            if(chisel_map)
            {
                const auto& chunkManager = chisel_map->GetChunkManager();
                chisel::Vec3 map_offset = chunkManager.GetOrigin();
                visualization_msgs::Marker marker;
                marker.header.stamp = ros::Time::now();
                marker.header.frame_id = "map";
                marker.id = id;
                marker.scale.x = 1;
                marker.scale.y = 1;
                marker.scale.z = 1;
                marker.pose.orientation.x = submap_transforms[submap_index].rotation().x();
                marker.pose.orientation.y = submap_transforms[submap_index].rotation().y();
                marker.pose.orientation.z = submap_transforms[submap_index].rotation().z();
                marker.pose.orientation.w = submap_transforms[submap_index].rotation().w();
                marker.pose.position.x = submap_transforms[submap_index].translation().x();
                marker.pose.position.y = submap_transforms[submap_index].translation().y();
                marker.pose.position.z = submap_transforms[submap_index].translation().z();
                marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
                const chisel::MeshMap& mesh_map = chisel_map->GetChunkManager().GetAllMeshes();
                FillMarkerTopicWithMeshes(mesh_map, &marker);
                if(marker.points.size() > 0)
                {
                    marker_array.markers.push_back(marker);
                    id++;
                }
            }
            submap_index++;
        }
        mesh_publisher_.publish(marker_array);
    }

}

chisel::Vec3 LAMBERT(const chisel::Vec3& n, const chisel::Vec3& light)
{
    return fmax(n.dot(light), 0.0f) * chisel::Vec3(0.5, 0.5, 0.5);
}

void Node::FillMarkerTopicWithMeshes(const chisel::MeshMap& meshMap, visualization_msgs::Marker* marker, int idx)
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
                  if(idx == -1)
                  {
                      color.r = fmin(lambert[0], 1.0);
                      color.g = fmin(lambert[1], 1.0);
                      color.b = fmin(lambert[2], 1.0);
                      color.a = 1.0;
                  }
                  else
                  {
                      color.r = (0.3+0.7*(((1+idx)%3)/2))*fmin(lambert[0], 1.0);
                      color.g = (0.3+0.7*(((2+idx)%5)/4))*fmin(lambert[1], 1.0);
                      color.b = (0.3+0.7*(((4+idx)%9)/8))*fmin(lambert[2], 1.0);
                      color.a = 1.0;
                  }
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
        }
    }
}

}  // namespace cartographer_ros
