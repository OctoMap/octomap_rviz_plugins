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
  , newPointsReceived_ (false)
  , messages_received_(0)
  , tf_filter_( 0 )
  , queue_size_(5)
  , treeDepth_(16)
  , colorFactor_(0.8)
{

  octomap_topic_property_ = new RosTopicProperty("Binary Octomap Topic", "",
                                                 QString::fromStdString(ros::message_traits::datatype<octomap_msgs::OctomapBinary>()),
                                                 "octomap_msgs::OctomapBinary topic to subscribe to.", this, SLOT( updateTopic() ));

  queue_size_property_ = new IntProperty( "Queue Size", queue_size_,
                                          "Advanced: set the size of the incoming message queue.  Increasing this "
                                          "is useful if your incoming TF data is delayed significantly from your"
                                          " image data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));
  queue_size_property_->setMin( 1 );


  tree_depth_property_ = new IntProperty("Max. Tree Depth",
                                         treeDepth_,
                                         "Defines the maximum tree depth",
                                         this,
                                            SLOT (updateTreeDepth() ));

}

void OctomapCloudDisplay::onInitialize()
{

  int i;

  pointcloud_scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  pointcloud_scene_node_->setVisible( true );

  // allocate point vectors for every tree depths
  newPoints_.resize(16);
  pointBuf_.resize(16);

  for (i=0; i<16; ++i)
  {
    std::stringstream sname;
    sname << "PointCloud Nr."<<i;
    cloud_[i] = new rviz::PointCloud();
    cloud_[i]->setName(sname.str());
    cloud_[i]->setRenderMode( PointCloud::RM_BOXES );
    pointcloud_scene_node_->attachObject(cloud_[i]);
  }

  tf_filter_ = new tf::MessageFilter<octomap_msgs::OctomapBinary>( *context_->getTFClient(), fixed_frame_.toStdString(),
                                                                       queue_size_property_->getInt(), threaded_nh_ );

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&OctomapCloudDisplay::incomingMessageCallback, this, _1));
  context_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);


}

OctomapCloudDisplay::~OctomapCloudDisplay()
{
  size_t i;

  unsubscribe();

  for (i=0; i<16; ++i)
  {
    pointcloud_scene_node_->detachAllObjects();
    delete cloud_[i];
  }

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
    std::string topicStr = octomap_topic_property_->getStdString();

    if (!topicStr.empty()) {

      // subscribe to depth map topic
      sub_.subscribe( threaded_nh_, topicStr, queue_size_ );

    }
  }
  catch (ros::Exception& e)
  {
    updateStatus( StatusProperty::Error, "Topic", (std::string("Error subscribing: ") + e.what()).c_str());
  }

}

void OctomapCloudDisplay::unsubscribe()
{
  clear();

  try
  {
    // reset filters
    sub_.unsubscribe( );
  }
  catch (ros::Exception& e)
  {
    updateStatus( StatusProperty::Error, "Topic", (std::string("Error unsubscribing: ") + e.what()).c_str());
  }

}

void OctomapCloudDisplay::setStatusList( )
{
  boost::mutex::scoped_lock lock(status_mutex_);

  QList<StatusListEntry>::iterator i;
  QList<StatusListEntry>::iterator i_end = statusList_.end();

  for (i = statusList_.begin(); i != i_end; ++i)
  {
    setStatus(i->level, i->name, i->text);
  }

  statusList_.clear();
}

void OctomapCloudDisplay::updateStatus( StatusProperty::Level level, const QString& name, const QString& text )
{
  boost::mutex::scoped_lock lock(status_mutex_);

  StatusListEntry newQueueEntry;

  newQueueEntry.level = level;
  newQueueEntry.name = name;
  newQueueEntry.text = text;

  statusList_.push_back(newQueueEntry);
}

// method taken from octomap_server package
void OctomapCloudDisplay::setColor( double z_pos, double min_z, double max_z, double color_factor, PointCloud::Point& point)
{
  int i;
  double m, n, f;

  double s = 1.0;
  double v = 1.0;

  double h = (1.0 - std::min(std::max((z_pos-min_z)/ (max_z - min_z), 0.0), 1.0)) *color_factor;

  h -= floor(h);
  h *= 6;
  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      point.setColor(v, n, m);
      break;
    case 1:
      point.setColor(n, v, m);
      break;
    case 2:
      point.setColor(m, v, n);
      break;
    case 3:
      point.setColor(m, n, v);
      break;
    case 4:
      point.setColor(n, m, v);
      break;
    case 5:
      point.setColor(v, m, n);
      break;
    default:
      point.setColor(1, 0.5, 0.5);
      break;
  }
}


void OctomapCloudDisplay::incomingMessageCallback( const octomap_msgs::OctomapBinaryConstPtr& msg )
{
  ++messages_received_;
  updateStatus(StatusProperty::Ok, "Messages", QString::number(messages_received_) + " binary octomap messages received");

  // creating octree from OctomapBinary message
  octomap::OcTree* octomap = NULL;
  octomap = octomap_msgs::binaryMsgDataToMap(msg->data);

  if (!octomap)
  {
    this->setStatusStd(StatusProperty::Error, "Message", "Failed to create octree structure");
    return;
  }

  // get dimensions of octree
  double minX, minY, minZ, maxX, maxY, maxZ;
  octomap->getMetricMin(minX, minY, minZ);
  octomap->getMetricMax(maxX, maxY, maxZ);


  // get tf transform
  Ogre::Matrix4 transform = Ogre::Matrix4::ZERO;
  Ogre::Vector3 pos;
  Ogre::Quaternion orient;
  if (!context_->getFrameManager()->getTransform(msg->header, pos, orient))
  {
    std::stringstream ss;
    ss << "Failed to transform from frame [" << msg->header.frame_id << "] to frame ["
        << context_->getFrameManager()->getFixedFrame() << "]";
    this->setStatusStd(StatusProperty::Error, "Message", ss.str());
    return;
  }
  transform = Ogre::Matrix4(orient);
  transform.setTrans(pos);

  // reset rviz pointcloud classes
  for (size_t i=0; i<16; ++i)
  {
    pointBuf_[i].clear();
    boxSizes_[i] = octomap->getNodeSize(i+1);
  }

  size_t pointCount = 0;
  {
    // traverse all leafs in the tree:
    unsigned int treeDepth = std::min<unsigned int>(treeDepth_, octomap->getTreeDepth());
    for (octomap::OcTreeROS::OcTreeType::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
    {

      if (octomap->isNodeOccupied(*it))
      {

        // check if current voxel has neighbors on all sides -> no need to be displayed
        bool allNeighborsFound = true;

        octomap::OcTreeKey key;
        octomap::OcTreeKey nKey = it.getKey();

        for (key[2] = nKey[2] - 1; allNeighborsFound && key[2] <= nKey[2] + 1; ++key[2])
        {
          for (key[1] = nKey[1] - 1; allNeighborsFound && key[1] <= nKey[1] + 1; ++key[1])
          {
            for (key[0] = nKey[0] - 1; allNeighborsFound && key[0] <= nKey[0] + 1; ++key[0])
            {
              if (key != nKey)
              {
                octomap::OcTreeNode* node = octomap->search(key);
                if (!(node && octomap->isNodeOccupied(node)))
                {
                  // we do not have a neighbor => break!
                  allNeighborsFound = false;
                }
              }
            }
          }

        }

        if (!allNeighborsFound)
        {
          PointCloud::Point newPoint;
          Ogre::Vector3 transPoint (it.getX(), it.getY(), it.getZ());
          // transform point according to tf frame
          transPoint = transform * transPoint;

          newPoint.x = transPoint.x;
          newPoint.y = transPoint.y;
          newPoint.z = transPoint.z;

          // apply color
          setColor(newPoint.z, minZ, maxZ, colorFactor_, newPoint);

          // push to point vectors
          unsigned int depth = it.getDepth();
          pointBuf_[depth-1].push_back(newPoint);

          ++pointCount;
        }
      }
    }
  }

  if (pointCount)
  {
    boost::mutex::scoped_lock lock(mutex_);

    newPointsReceived_ = true;

    for (size_t i=0; i<16; ++i)
    {
      newPoints_[i].clear();
      newPoints_[i].resize(pointBuf_[i].size());
      std::copy(pointBuf_[i].begin(), pointBuf_[i].end(), newPoints_[i].begin());
    }

  }
  delete octomap;
}

void OctomapCloudDisplay::updateTreeDepth()
{
  treeDepth_ = tree_depth_property_->getInt();
}

void OctomapCloudDisplay::clear()
{

  boost::mutex::scoped_lock lock(mutex_);

  // reset rviz pointcloud boxes
  for (size_t i=0; i<16; ++i)
  {
    cloud_[i]->clear();
  }

}

void OctomapCloudDisplay::update(float wall_dt, float ros_dt)
{




  if (newPointsReceived_)
  {
    boost::mutex::scoped_lock lock(mutex_);

    for (size_t i=0; i<16; ++i)
    {
      double size = boxSizes_[i];

      cloud_[i]->clear();
      cloud_[i]->setDimensions( size, size, size );

      cloud_[i]->addPoints(&newPoints_[i].front(), newPoints_[i].size());
      newPoints_[i].clear();

    }
    newPointsReceived_ = false;
  }

  setStatusList();
}

void OctomapCloudDisplay::reset()
{
  clear();
  messages_received_ = 0;
  updateStatus(StatusProperty::Ok, "Messages", QString("0 binary octomap messages received"));
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

} // namespace octomap_rviz_plugin

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( octomap_rviz_plugin, OctomapCloud, octomap_rviz_plugin::OctomapCloudDisplay, rviz::Display)

