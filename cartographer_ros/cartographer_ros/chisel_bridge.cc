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

#include "cartographer_ros/chisel_bridge.h"

#include "cartographer_ros/assets_writer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/occupancy_grid.h"
#include "cartographer_ros_msgs/TrajectorySubmapList.h"
#include "chisel_msgs/IncrementalChangesMessage.h"
#include "chisel_msgs/VolumeMessage.h"
#include "chisel_mason_plugins/chisel_conversions.h"


namespace cartographer_ros {
ChiselBridge::ChiselBridge()
{}

void chunkToMsg(chisel::ChunkConstPtr<chisel::DistVoxel> chunk, chisel_msgs::ChunkMessage& msg)
{
  chisel::ChunkHasher hasher;
  //msg.header = header;

  const chisel::ChunkID& id = chunk->GetID();
  const Eigen::Vector3i& size = chunk->GetNumVoxels();
  const std::vector<chisel::DistVoxel>& voxels = chunk->GetVoxels();

  msg.ID_x = id.x();
  msg.ID_y = id.y();
  msg.ID_z = id.z();
  msg.spatial_hash = hasher(id);

  msg.resolution_meters = chunk->GetVoxelResolutionMeters();

  msg.num_voxels_x = size.x();
  msg.num_voxels_y = size.y();
  msg.num_voxels_z = size.z();

  for (const chisel::DistVoxel& voxel : voxels)
  {
    msg.sdf.push_back(voxel.GetSDF());
    msg.weights.push_back(voxel.GetWeight());
  }

  if (chunk->HasColors())
  {
    const std::vector<chisel::ColorVoxel>& colors = chunk->GetColorVoxels();

    for (const chisel::ColorVoxel& voxel : colors)
    {
      msg.red.push_back(voxel.GetRed());
      msg.blue.push_back(voxel.GetBlue());
      msg.green.push_back(voxel.GetGreen());
      msg.color_weight.push_back(voxel.GetWeight());
    }
  }
}

void ChiselBridge::PublishTSDF(MapBuilderBridge* map_builder_bridge) {

  std::vector<chisel::ChiselPtr<chisel::DistVoxel>> tsdf_list = map_builder_bridge->GetTSDFList();
  int trajectory_id = 0;
  const cartographer::mapping::Submaps* const submaps =
      map_builder_bridge->map_builder_.GetTrajectoryBuilder(trajectory_id)->submaps();

  const std::vector<cartographer::transform::Rigid3d> submap_transforms =
      map_builder_bridge->map_builder_.sparse_pose_graph()->GetSubmapTransforms(trajectory_id);

  if(tsdf_list.size() > 0 && tsdf_pointcloud_publisher_.getNumSubscribers() > 0){
    //todo(kdaun) add global transform for loop closure correction
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.clear();
    int submap_index = 0;
    for(const chisel::ChiselPtr<chisel::DistVoxel> chisel_map : tsdf_list)
    {
      if (submap_index < 0 || submap_index >= submaps->size()) {
        ROS_INFO( "Requested submap %i from trajectory %i but there are only %i submaps in this trajectory.",
                  submap_index, trajectory_id, submaps->size());
      }
      if(chisel_map)
      {
        const auto& chunkManager = chisel_map->GetChunkManager();
        const float resolution = chunkManager.GetResolution();
        chisel::Vec3 map_offset = chunkManager.GetOrigin();

        int stepSize = 1;

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
    pc.header.frame_id = "world";
    pc.header.stamp = ros::Time::now();
    tsdf_pointcloud_publisher_.publish(pc);
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
        //const auto& chunkManager = chisel_map->GetChunkManager();
        //chisel::Vec3 map_offset = chunkManager.GetOrigin();
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "world";
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
        marker.header.frame_id = "world";
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
        //const auto& chunkManager = chisel_map->GetChunkManager();
        //chisel::Vec3 map_offset = chunkManager.GetOrigin();
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "world";
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


  if(tsdf_list.size() > 0 && volume_publisher_.getNumSubscribers() > 0){
    //chisel_msgs::VolumeMessage volume_msg;
    const chisel::ChiselPtr<chisel::DistVoxel> chisel_map = tsdf_list[0];
    chisel_msgs::VolumeMessagePtr volume_msg = boost::make_shared<chisel_msgs::VolumeMessage>();
    chisel_msgs::IncrementalChangesMessagePtr inc_changes_msg = boost::make_shared<chisel_msgs::IncrementalChangesMessage>();

    volume_msg->header.stamp = ros::Time::now();
    volume_msg->header.frame_id = "world";

    inc_changes_msg->header.stamp = volume_msg->header.stamp;
    inc_changes_msg->header.frame_id = "world";


    if(chisel_map)
    {
      chisel::ChunkManager<chisel::DistVoxel>& chunk_manager = chisel_map->GetMutableChunkManager();
      const chisel::ChunkMap<chisel::DistVoxel>& chunk_map = chunk_manager.GetChunks();
      if (chunk_map.size() > 0)
      {
        //volume_msg->header = header;

        for ( auto it = chunk_map.begin(); it != chunk_map.end(); ++it )
        {
          chisel_msgs::ChunkMessage msg;
          chunkToMsg(it->second, msg);
          volume_msg->chunk_list.chunks.push_back(msg);
        }
      }


      inc_changes_msg->num_voxels_x = chunk_manager.GetChunkSize()(0);
      inc_changes_msg->num_voxels_y = chunk_manager.GetChunkSize()(1);
      inc_changes_msg->num_voxels_z = chunk_manager.GetChunkSize()(2);
      inc_changes_msg->resolution_meters = chunk_manager.GetResolution();

      chisel::IncrementalChangesConstPtr<chisel::DistVoxel> incremental_changes_ = boost::make_shared<chisel::IncrementalChanges<chisel::DistVoxel>>(*chunk_manager.getIncrementalChanges());

      mason::chiselChunkMapToMsg(incremental_changes_->addedChunks, inc_changes_msg->added_chunks); /// TODO: What's about voxel changes here?

      mason::chiselChunkMapToMsg(incremental_changes_->updatedChunks, inc_changes_msg->updated_chunks);
      mason::chiselCarvedVoxelsToMsg(incremental_changes_->carvedVoxels, inc_changes_msg->updated_chunks);
      mason::chiselUpdatedVoxelsToMsg(incremental_changes_->updatedVoxels, inc_changes_msg->updated_chunks);

      mason::chiselChunkSetToMsg(incremental_changes_->deletedChunks, inc_changes_msg->deleted_chunks);


      incremental_changes_publisher_.publish(inc_changes_msg);
      volume_publisher_.publish(volume_msg);
      chunk_manager.clearIncrementalChanges();
    }



  }

}

chisel::Vec3 LAMBERT(const chisel::Vec3& n, const chisel::Vec3& light)
{
  return fmax(n.dot(light), 0.0f) * chisel::Vec3(0.5, 0.5, 0.5);
}

void ChiselBridge::FillMarkerTopicWithMeshes(const chisel::MeshMap& meshMap, visualization_msgs::Marker* marker, int idx)
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
