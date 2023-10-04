/**
 * MapMerger constructor. Initializes ROS node handles, parameters, 
 * RTAB-Map instance, and ROS publishers/subscribers.
 *
 * Parameters:
 * - map1_topic: ROS topic for first input map (default "none")
 * - map2_topic: ROS topic for second input map (default "none") 
 * - combo_map_topic: ROS topic to publish merged map (default "comboMapData")
 * - base_frame: Frame ID for merged map (default "map")
 * - db_location: Database location for RTAB-Map (default "")
 * - config_path: Path to RTAB-Map config file (default "") 
 * - merge_freq: Frequency (Hz) for map merge timer callback (default 10)
 * - odom_linear_variance: Odometry linear variance for RTAB-Map (default 0.0001)
 * - odom_angular_variance: Odometry angular variance for RTAB-Map (default 0.0005)
 * - merge_map_optimized: Optimize graph before extracting map (default false)
 * - merge_map_global: Extract global-optimized poses (default true)
 * 
 * Subscribes to:
 * - map1_topic 
 * - map2_topic
 *
 * Publishes to: 
 * - combo_map_topic
 *
 * Initializes RTAB-Map with parameters and sets up map merge callbacks.
 *
 * based on Jacob Olson's work 
 */

#include "map_merger.h"
#include <ros/ros.h>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>

#include <rtabmap_msgs/MapData.h>
#include <rtabmap_msgs/NodeData.h>
#include <rtabmap_util/MapsManager.h>
#include <rtabmap_conversions/MsgConversion.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UFile.h>

using namespace std;
using namespace rtabmap;
using namespace rtabmap_msgs;
using namespace rtabmap_util;
using namespace rtabmap_conversions;

namespace map_merge
{
  MapMerger::MapMerger() :
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~")),
    map_to_odom_(Transform::getIdentity()),
    map1_len_(0),
    map2_len_(0),
    update_now_(true)
  {
    
    // Topic Params
    nh_private_.param<string>("map1_topic", map1_topic_, "none");
    nh_private_.param<string>("map2_topic", map2_topic_, "none");
    nh_private_.param<string>("combined_map_topic", combo_map_topic_, "comboMapData");

    // Other ROS params
    nh_private_.param<string>("base_frame", base_frame_, "map");
    nh_private_.param<string>("db_location", db_location_, db_location_);
    nh_private_.param<string>("config_path", config_path_, config_path_);
    nh_private_.param<double>("map_merge_frequency", merge_freq_, 10);
    nh_private_.param<float>("odom_linear_variance", odom_linear_variance_, 0.0001f);
    nh_private_.param<float>("odom_angular_variance", odom_angular_variance_, 0.0005f);
    nh_private_.param<bool>("merge_map_optimized", merge_map_optimized_, false);
    nh_private_.param<bool>("merge_map_global", merge_map_global_, true);

    // Initialize rtabmap instance and maps manager
    setupRtabParams();
    maps_manager_.init(nh_, nh_private_, "map_merge_manager", true);

    // Setup ROS hooks
    map1_subscriber_ = nh_.subscribe(map1_topic_, 1, &MapMerger::mapCallback1, this);
    map2_subscriber_ = nh_.subscribe(map2_topic_, 1, &MapMerger::mapCallback2, this);
    update_map_timer_ = nh_.createTimer(ros::Duration(merge_freq_), &MapMerger::timerCallback, this);

    map_data_pub_ = nh_.advertise<MapData>(combo_map_topic_, 1);
    cout << "MAP TOPICS: " << map1_topic_ << ", " << map2_topic_ << endl;
  }

  MapMerger::~MapMerger()
  {
    // Close and save the RTAB-Map database
    //rtabmap_.close(); // Close the database and save it

    // Optionally, print a message indicating successful closure
    //std::cout << "RTAB-Map database closed and saved." << std::endl;
  }

  void MapMerger::timerCallback(const ros::TimerEvent& msg) 
  {
     
    cout << "timer called" << endl;
    if (update_now_) 
    {
      std::unique_lock<std::mutex> lock(mutex_);
      stringstream nodes_message;
      nodes_message << "Merging Maps, Map1 Nodes: " << nodes_map1.size();
      temp_nodes_map1 = nodes_map1;
      if (map2_) 
      {
        nodes_message << ", Map2 Nodes:" << nodes_map2.size();
        temp_nodes_map2 = nodes_map2;
      }
      nodes_message << endl;
      cout << nodes_message.str();

      std::thread merge_thread(&MapMerger::mergeMaps, this); // multi threaded mergeMaps
      merge_thread.detach();
    }
  }

  void MapMerger::mergeMaps() 
  {
    // Merging request, process each map one after the other
    update_now_ = false;
    if (map1_) 
    {
        for (auto iter = std::next(temp_nodes_map1.begin(), std::max(0, map1_len_ - 5)); iter != temp_nodes_map1.end(); ++iter) {
            SensorData data = iter->sensorData();
            data.uncompressData();
            rtabmap_.process(data, iter->getPose(), odom_linear_variance_, odom_angular_variance_);
            map1_len_++;
        }
        cout << "Scanned Map 1: " << temp_nodes_map1.size() << " nodes in map." << endl;
    }

    if (map2_) 
    {
        for (auto iter = std::next(temp_nodes_map2.begin(), std::max(0, map2_len_ - 5)); iter != temp_nodes_map2.end(); ++iter) {
            SensorData data = iter->sensorData();
            data.uncompressData();
            rtabmap_.process(data, iter->getPose(), odom_linear_variance_, odom_angular_variance_);
            map2_len_++;
        }
        cout << "Scanned Map 2: " << temp_nodes_map2.size() << " nodes in map." << endl;
    }

    // Update and publish combined map
    maps_manager_.clear();
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> constraints;
    rtabmap_.getGraph(poses, constraints, true, true);
    poses = maps_manager_.updateMapCaches(poses, rtabmap_.getMemory(), false, false);

    // Map is published as pointcloud2 /cloud_map connected to the base_frame_
    maps_manager_.publishMaps(poses, ros::Time::now(), base_frame_);

    // Publish map as MapData Topic which allows for setting max ceiling height
    // and floor height to improve map readability
    publishComboMapData();
    update_now_ = true;
  }

  void MapMerger::mapCallback1(const MapData& msg) 
  {
    
    std::unique_lock<std::mutex> lock(mutex_);
    NodeData node{msg.nodes.back()};

    // Create a Signature directly from NodeData
    Signature sig{nodeDataFromROS(node)};

    nodes_map1.push_back(sig);
    // lock is automatically released when lock goes out of scope
  }

  void MapMerger::mapCallback2(const MapData& msg) 
  {
   
    std::unique_lock<std::mutex> lock(mutex_);
    NodeData node{msg.nodes.back()};

    // Create a Signature directly from NodeData
    Signature sig{nodeDataFromROS(node)};

    nodes_map2.push_back(sig);
    // lock is automatically released when lock goes out of scope
  }

  void MapMerger::setupRtabParams() 
  {
    // Parameters
    ParametersMap parameters = Parameters::getDefaultParameters();
    parameters.at(Parameters::kDbSqlite3InMemory()) = "true";
    parameters.at(Parameters::kMemReduceGraph()) = "true";

    rtabmap_.init(parameters, db_location_);
    // Set up map booleans
    map1_ = true;
    map2_ = true;
  }

  void MapMerger::publishComboMapData()
  {
    std::map<int, rtabmap::Transform> poses_md;
    std::multimap<int, rtabmap::Link> constraints_md;
    std::map<int, rtabmap::Signature> signatures_md;

    // Set _md variables from map date created in mergeMaps()
    rtabmap_.get3DMap(signatures_md,
                      poses_md,
                      constraints_md,
                      merge_map_optimized_,
                      merge_map_global_);

    // setup mapdata message
    MapDataPtr msg(new MapData);
    ros::Time now = ros::Time::now();
    msg->header.stamp = now;
    msg->header.frame_id = base_frame_;

    // push mapdata to ros message
    mapDataToROS(poses_md,
                 constraints_md,
                 signatures_md,
                 map_to_odom_,
                 *msg);

    // publish message
    map_data_pub_.publish(msg);
  }

  

} // namespace map_merger