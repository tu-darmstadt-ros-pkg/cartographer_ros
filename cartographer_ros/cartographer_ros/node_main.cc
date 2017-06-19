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

#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/String.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions() {
    auto file_resolver = cartographer::common::make_unique<
            cartographer::common::ConfigurationFileResolver>(
                std::vector<string>{FLAGS_configuration_directory});
    const string code =
            file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
    cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
                code, std::move(file_resolver));

    return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                           CreateTrajectoryOptions(&lua_parameter_dictionary));
}

class NodeWrapper
{

public:
    std::unique_ptr<tf2_ros::TransformListener> tf;
    std::unique_ptr<Node> node;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    ros::Subscriber syscommand_subscriber;
    ros::NodeHandle node_handle_;

    NodeWrapper()
    {
        syscommand_subscriber = node_handle_.subscribe("syscommand", 1000, &NodeWrapper::SyscommandCallback, this);
    }



    void Run() {
        constexpr double kTfBufferCacheTimeInSeconds = 1e6;
        tf_buffer = cartographer::common::make_unique<tf2_ros::Buffer>(::ros::Duration(kTfBufferCacheTimeInSeconds));
        //tf2_ros::TransformListener tf(tf_buffer);
        tf = cartographer::common::make_unique<tf2_ros::TransformListener>(*tf_buffer);
        NodeOptions node_options;
        TrajectoryOptions trajectory_options;
        std::tie(node_options, trajectory_options) = LoadOptions();

        //Node node(node_options, &tf_buffer);

        node = cartographer::common::make_unique<Node>(node_options, tf_buffer.get());
        node->StartTrajectoryWithDefaultTopics(trajectory_options);

    }

    void Finish()
    {
        node->FinishAllTrajectories();
    }

    void SyscommandCallback(const std_msgs::String::ConstPtr& msg)
    {
        if(msg->data == "reset" || msg->data == "reset_map")
        {
            ROS_INFO("Received2: %s", msg->data.c_str());
            //map_builder_bridge_->map_builder_.reset();
            //tf2_ros::Buffer* tf_buffer;// = map_builder_bridge_->tf_buffer_;
            // map_builder_bridge_ = MapBuilderBridge(node_options_, tf_buffer);
            //map_builder_bridge_.reset();
            //map_builder_bridge_ = std::make_shared<MapBuilderBridge>(node_options_, tf_buffer);
            /*      FinishAllTrajectories();
        {
          carto::common::MutexLocker lock(&mutex_);
          terminating_ = true;
        }
        if (occupancy_grid_thread_.joinable()) {
          occupancy_grid_thread_.join();
        }
*/
            Finish();
            node.reset();
            tf.reset();
            tf_buffer.reset();
            Run();
        }
    }



};  // namespace
}  // namespace cartographer_ros
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_configuration_directory.empty())
            << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basename.empty())
            << "-configuration_basename is missing.";

    ::ros::init(argc, argv, "cartographer_node");
    ::ros::start();

    cartographer_ros::ScopedRosLogSink ros_log_sink;

    cartographer_ros::NodeWrapper node_wrapper;
    node_wrapper.Run();
    ::ros::spin();
    node_wrapper.Finish();
    ::ros::shutdown();
}
