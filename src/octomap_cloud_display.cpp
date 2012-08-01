/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
#include <QObject>

#include "octomap_cloud_display.h"

#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/validate_floats.h"
#include "rviz/frame_manager.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <sstream>

using namespace rviz;


namespace octomap_rviz_plugin
{

  OctomapCloudDisplay::OctomapCloudDisplay()
  : rviz::Display()
  , messages_received_(0)
  , tf_filter_( 0 )
  , queue_size_(5)
{

  // Depth map properties
    octomap_topic_property_ = new RosTopicProperty("Compressed Octomap Topic", "",
                                         QString::fromStdString(ros::message_traits::datatype<octomap_msgs::OctomapBinary>()),
                                         "octomap_msgs::OctomapBinary topic to subscribe to.", this, SLOT( updateTopic() ));

  // Queue size property
  queue_size_property_ = new IntProperty( "Queue Size", queue_size_,
                                          "Advanced: set the size of the incoming message queue.  Increasing this "
                                          "is useful if your incoming TF data is delayed significantly from your"
                                          " image data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));
  queue_size_property_->setMin( 1 );


  // Instantiate PointCloudCommon class for displaying point clouds
  pointcloud_common_ = new PointCloudCommon(this);

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.
  update_nh_.setCallbackQueue( pointcloud_common_->getCallbackQueue() );

}

void OctomapCloudDisplay::onInitialize()
{

  pointcloud_scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  pointcloud_scene_node_->setVisible( true );

  pointcloud_common_->initialize(context_, pointcloud_scene_node_);

  tf_filter_ = new tf::MessageFilter<octomap_msgs::OctomapBinary>( *context_->getTFClient(), fixed_frame_.toStdString(),
                                                                       queue_size_property_->getInt(), update_nh_ );

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&OctomapCloudDisplay::incomingMessageCallback, this, _1));
  context_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);

}

OctomapCloudDisplay::~OctomapCloudDisplay()
{

  unsubscribe();

  if (pointcloud_common_)
    delete pointcloud_common_;

  if (tf_filter_)
    delete tf_filter_;


}
void OctomapCloudDisplay::updateQueueSize()
{
  queue_size_ = queue_size_property_->getInt();

  if (tf_filter_)
    tf_filter_->setQueueSize(queue_size_);
}

void OctomapCloudDisplay::onEnable()
{
  pointcloud_scene_node_->setVisible( true );
  subscribe();
}

void OctomapCloudDisplay::onDisable()
{
  pointcloud_scene_node_->setVisible( false );
  unsubscribe();

  clear();
}

void OctomapCloudDisplay::subscribe()
{
  if( !isEnabled() )
  {
    return;
  }

  try
  {
    boost::mutex::scoped_lock lock(mutex_);

    std::string topicStr = octomap_topic_property_->getStdString();

    if (!topicStr.empty()) {

      // subscribe to depth map topic
      sub_.subscribe( update_nh_, topicStr, queue_size_ );
    }
  }
  catch (ros::Exception& e)
  {
    setStatus( StatusProperty::Error, "Topic", (std::string("Error subscribing: ") + e.what()).c_str());
  }

}

void OctomapCloudDisplay::incomingMessageCallback( const octomap_msgs::OctomapBinaryConstPtr& msg )
{
  ++messages_received_;
  setStatus(StatusProperty::Ok, "Messages", QString::number(messages_received_) + " binary octomap messages received");

  // creating octree from OctomapBinary message
  octomap::OcTree octomap(0.1);
  octomap::octomapMsgToMap(*msg, octomap);

  // output pointcloud2 message
  sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);

  cloud_msg->header = msg->header;
  cloud_msg->fields.resize(3);
  cloud_msg->fields[0].name = "x";
  cloud_msg->fields[1].name = "y";
  cloud_msg->fields[2].name = "z";

  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < cloud_msg->fields.size(); ++d, offset += 4)
  {
    cloud_msg->fields[d].offset = offset;
    cloud_msg->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg->fields[d].count = 1;
  }

  cloud_msg->point_step = offset;

  size_t numLeafs = octomap.getNumLeafNodes();

  cloud_msg->data.resize(numLeafs * offset);
  cloud_msg->is_bigendian = false;
  cloud_msg->is_dense = false;

  unsigned int m_maxTreeDepth = 16;

  float* cloudDataPtr = reinterpret_cast<float*>(&cloud_msg->data[0]);
  size_t pointCount = 0;

  // traverse all leafs in the tree:
  for (octomap::OcTreeROS::OcTreeType::iterator it = octomap.begin(m_maxTreeDepth), end = octomap.end(); it != end; ++it)
  {

    if (octomap.isNodeOccupied(*it))
    {
      *cloudDataPtr = it.getX();  ++cloudDataPtr;
      *cloudDataPtr = it.getY();  ++cloudDataPtr;
      *cloudDataPtr = it.getZ();  ++cloudDataPtr;

      ++pointCount;
    }
  }

  ////////////////////////////////////////////////
  // finalize pointcloud2 message
  ////////////////////////////////////////////////
  cloud_msg->width = pointCount;
  cloud_msg->height = 1;
  cloud_msg->data.resize(pointCount * cloud_msg->point_step);
  cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;

  // add generated point cloud to pointcloud_common
  pointcloud_common_->addMessage(cloud_msg);

}

void OctomapCloudDisplay::unsubscribe()
{
  clear();

  boost::mutex::scoped_lock lock(mutex_);

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

void OctomapCloudDisplay::clear()
{

  boost::mutex::scoped_lock lock(mutex_);

  pointcloud_common_->reset();
}

void OctomapCloudDisplay::update(float wall_dt, float ros_dt)
{

  boost::mutex::scoped_lock lock(mutex_);

  pointcloud_common_->update(wall_dt, ros_dt);

}

void OctomapCloudDisplay::reset()
{
  clear();
  messages_received_ = 0;
  setStatus(StatusProperty::Ok, "Messages", QString("0 binary octomap messages received"));
}

void OctomapCloudDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void OctomapCloudDisplay::fixedFrameChanged()
{
  Display::reset();
}

} // namespace depth_cloud_plugin

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( octomap_rviz_plugin, OctomapCloud, octomap_rviz_plugin::OctomapCloudDisplay, rviz::Display)

