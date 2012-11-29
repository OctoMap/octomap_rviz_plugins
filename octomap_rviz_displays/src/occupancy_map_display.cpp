/*
 * Copyright (c) 2008, Willow Garage, Inc.
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


#include "occupancy_map_display.h"

#include "rviz/visualization_manager.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/validate_floats.h"
#include "rviz/frame_manager.h"

using namespace rviz;

namespace octomap_rviz_plugin
{

OccupancyMapDisplay::OccupancyMapDisplay()
  : rviz::MapDisplay()
  , tf_filter_( 0 )
  , max_octree_depth_ (16)
{
  topic_property_->setName("Octomap Binary Topic");
  topic_property_->setMessageType( QString::fromStdString(ros::message_traits::datatype<octomap_msgs::OctomapBinary>()) );
  topic_property_->setDescription( "Octomap binary topic to subscribe to." );

  tree_depth_property_ = new IntProperty("Max. Octree Depth", max_octree_depth_, "Defines the maximum tree depth", this,
                                         SLOT (updateTreeDepth() ));
}

OccupancyMapDisplay::~OccupancyMapDisplay()
{
  unsubscribe();

  if (tf_filter_)
    delete tf_filter_;
}

void OccupancyMapDisplay::onInitialize()
{
  rviz::MapDisplay::onInitialize();

  tf_filter_ = new tf::MessageFilter<octomap_msgs::OctomapBinary>( *context_->getTFClient(), fixed_frame_.toStdString(), 1, threaded_nh_ );

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&OccupancyMapDisplay::handleOctomapBinaryMessage, this, _1));
  context_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

void OccupancyMapDisplay::updateTreeDepth()
{
  max_octree_depth_ = tree_depth_property_->getInt();
}

void OccupancyMapDisplay::subscribe()
{
  if( !isEnabled() )
  {
    return;
  }

  try
  {
    std::string topicStr = topic_property_->getStdString();

    if (!topicStr.empty()) {

      // subscribe to depth map topic
      sub_.subscribe( threaded_nh_, topicStr, 1 );

    }
  }
  catch (ros::Exception& e)
  {
    setStatus( StatusProperty::Error, "Topic", (std::string("Error subscribing: ") + e.what()).c_str());
  }
}

void OccupancyMapDisplay::unsubscribe()
{
  clear();

  try
  {
    // reset filters
    sub_.unsubscribe( );
  }
  catch (ros::Exception& e)
  {
    setStatus( StatusProperty::Error, "Topic", (std::string("Error unsubscribing: ") + e.what()).c_str());
  }
}


void OccupancyMapDisplay::handleOctomapBinaryMessage(const octomap_msgs::OctomapBinaryConstPtr& msg)
{
  // creating octree from OctomapBinary message
  octomap::OcTree* octomap = NULL;
  octomap = octomap_msgs::binaryMsgDataToMap(msg->data);

  ROS_DEBUG("Received OctomapBinary message (size: %d bytes)", (int)msg->data.size());

  if (!octomap)
  {
    this->setStatusStd(StatusProperty::Error, "Message", "Failed to create octree structure");
    return;
  }

  // get dimensions of octree
  double minX, minY, minZ, maxX, maxY, maxZ;
  octomap->getMetricMin(minX, minY, minZ);
  octomap->getMetricMax(maxX, maxY, maxZ);
  octomap::point3d minPt = octomap::point3d(minX, minY, minZ);

  unsigned int tree_depth = octomap->getTreeDepth();

  octomap::OcTreeKey paddedMinKey;
  octomap->genKey(minPt, paddedMinKey);

  nav_msgs::OccupancyGrid::Ptr occupancy_map (new nav_msgs::OccupancyGrid());

  unsigned int width, height;
  double res;

  unsigned int ds_shift = tree_depth-max_octree_depth_;

  occupancy_map->header = msg->header;
  occupancy_map->info.resolution = res = octomap->getNodeSize(max_octree_depth_);
  occupancy_map->info.width = width = (maxX-minX) / res + 1;
  occupancy_map->info.height = height = (maxY-minY) / res + 1;
  occupancy_map->info.origin.position.x = minX  - (res / (float)(1<<ds_shift) ) + res;
  occupancy_map->info.origin.position.y = minY  - (res / (float)(1<<ds_shift) );;

  occupancy_map->data.clear();
  occupancy_map->data.resize(width*height, -1);

    // traverse all leafs in the tree:
  for (octomap::OcTreeROS::OcTreeType::iterator it = octomap->begin(max_octree_depth_), end = octomap->end(); it != end; ++it)
  {
    bool occupied = octomap->isNodeOccupied(*it);
    int intSize = 1 << (max_octree_depth_ - it.getDepth());

    octomap::OcTreeKey minKey=it.getIndexKey();

    for (int dx = 0; dx < intSize; dx++)
    {
      for (int dy = 0; dy < intSize; dy++)
      {
        int posX = std::max<int>(0, minKey[0] + dx - paddedMinKey[0]);
        posX>>=ds_shift;

        int posY = std::max<int>(0, minKey[1] + dy - paddedMinKey[1]);
        posY>>=ds_shift;

        int idx = width * posY + posX;

        if (occupied)
          occupancy_map->data[idx] = 100;
        else if (occupancy_map->data[idx] == -1)
        {
          occupancy_map->data[idx] = 0;
        }

      }
    }

  }

  delete octomap;

  this->incomingMap(occupancy_map);
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( octomap_rviz_plugin, OccupancyMap, octomap_rviz_plugin::OccupancyMapDisplay, rviz::Display)

