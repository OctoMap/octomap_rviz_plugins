/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_OCCUPANCY_MAP_DISPLAY_H
#define RVIZ_OCCUPANCY_MAP_DISPLAY_H

#include <qobject.h>
#include "rviz/default_plugin/map_display.h"

#if ROS_VERSION_MINIMUM(1,8,0) // test for Fuerte (newer PCL)
  #include <octomap_msgs/OctomapBinary.h>
  #include <octomap_msgs/GetOctomap.h>
  #include <octomap_msgs/BoundingBoxQuery.h>
#else
  #include <octomap_ros/OctomapBinary.h>
  #include <octomap_ros/GetOctomap.h>
  #include <octomap_ros/ClearBBXRegion.h>
#endif

#include <octomap_ros/OctomapROS.h>
#include <octomap/OcTreeKey.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

using namespace rviz;

namespace octomap_rviz_plugin
{

/**
 * \class OccupancyMapDisplay
 *
 */
class OccupancyMapDisplay: public rviz::MapDisplay
{
Q_OBJECT
public:
  OccupancyMapDisplay();
  virtual ~OccupancyMapDisplay();

private Q_SLOTS:
  void updateTreeDepth();

protected:
  virtual void onInitialize();
  virtual void subscribe();
  virtual void unsubscribe();

  void handleOctomapBinaryMessage(const octomap_msgs::OctomapBinaryConstPtr& msg);

  message_filters::Subscriber<octomap_msgs::OctomapBinary> sub_;
  tf::MessageFilter<octomap_msgs::OctomapBinary>* tf_filter_;

  unsigned int max_octree_depth_;
  IntProperty* tree_depth_property_;

};

} // namespace rviz

 #endif
