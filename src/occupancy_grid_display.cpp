// Copyright (c) 2013, Willow Garage, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Julius Kammerl (jkammerl@willowgarage.com)

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <sstream>

#include <QObject>  // NOLINT

#include <octomap_rviz_plugins/occupancy_grid_display.hpp> // NOLINT

#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/status_property.hpp"

#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"
#include "octomap/OcTreeStamped.h"

#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"

namespace octomap_rviz_plugins
{
using rviz_common::properties::StatusProperty;

enum OctreeVoxelRenderMode
{
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2
};

enum OctreeVoxelColorMode
{
  OCTOMAP_CELL_COLOR,
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_PROBABILITY_COLOR,
};

OccupancyGridDisplay::OccupancyGridDisplay()
{
  octree_render_property_ = new rviz_common::properties::EnumProperty(
    "Voxel Rendering", "Occupied Voxels",
    "Select voxel type.",
    this,
    SLOT(updateOctreeRenderMode()) );

  octree_render_property_->addOption("Occupied Voxels", OCTOMAP_OCCUPIED_VOXELS);
  octree_render_property_->addOption("Free Voxels", OCTOMAP_FREE_VOXELS);
  octree_render_property_->addOption("All Voxels", OCTOMAP_FREE_VOXELS | OCTOMAP_OCCUPIED_VOXELS);

  octree_coloring_property_ = new rviz_common::properties::EnumProperty(
    "Voxel Coloring", "Z-Axis",
    "Select voxel coloring mode",
    this,
    SLOT(updateOctreeColorMode()) );

  octree_coloring_property_->addOption("Cell Color", OCTOMAP_CELL_COLOR);
  octree_coloring_property_->addOption("Z-Axis", OCTOMAP_Z_AXIS_COLOR);
  octree_coloring_property_->addOption("Cell Probability", OCTOMAP_PROBABILITY_COLOR);
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Voxel Alpha", 1.0, "Set voxel transparency alpha",
    this,
    SLOT(updateAlpha()) );
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  tree_depth_property_ = new rviz_common::properties::IntProperty(
    "Max. Octree Depth",
    MAX_OCTREE_DEPTH,
    "Defines the maximum tree depth",
    this,
    SLOT(updateTreeDepth()));
  tree_depth_property_->setMin(0);

  max_height_property_ = new rviz_common::properties::FloatProperty(
    "Max. Height Display",
    std::numeric_limits<double>::infinity(),
    "Defines the maximum height to display",
    this,
    SLOT(updateMaxHeight()));

  min_height_property_ = new rviz_common::properties::FloatProperty(
    "Min. Height Display",
    -std::numeric_limits<double>::infinity(),
    "Defines the minimum height to display",
    this,
    SLOT(updateMinHeight()));
}

void OccupancyGridDisplay::onInitialize()
{
  MFDClass::onInitialize();
  std::unique_lock<std::mutex> lock(mutex_);

  box_size_.resize(MAX_OCTREE_DEPTH);
  cloud_.resize(MAX_OCTREE_DEPTH);
  point_buf_.resize(MAX_OCTREE_DEPTH);
  new_points_.resize(MAX_OCTREE_DEPTH);

  for (std::size_t i = 0; i < MAX_OCTREE_DEPTH; ++i) {
    std::stringstream sname;
    sname << "PointCloud Nr." << i;
    cloud_[i] = std::make_shared<rviz_rendering::PointCloud>();
    cloud_[i]->setName(sname.str());
    cloud_[i]->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);
    scene_node_->attachObject(cloud_[i].get());
  }
}

void OccupancyGridDisplay::unsubscribe()
{
  clear();
  MFDClass::unsubscribe();
}

// method taken from octomap_server package
void OccupancyGridDisplay::setColor(
  double z_pos, double min_z, double max_z, double color_factor,
  rviz_rendering::PointCloud::Point & point)
{
  int i{};
  double m{};
  double n{};
  double f{};

  double s = 1.0;
  double v = 1.0;

  double h = (1.0 - std::min(std::max((z_pos - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor;

  h -= floor(h);
  h *= 6;
  i = floor(h);
  f = h - i;
  if (!(i & 1)) {
    f = 1 - f;  // if i is even
  }
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

void OccupancyGridDisplay::updateTreeDepth()
{
  MFDClass::updateTopic();
}

void OccupancyGridDisplay::updateOctreeRenderMode()
{
  MFDClass::updateTopic();
}

void OccupancyGridDisplay::updateOctreeColorMode()
{
  MFDClass::updateTopic();
}

void OccupancyGridDisplay::updateAlpha()
{
  MFDClass::updateTopic();
}

void OccupancyGridDisplay::updateMaxHeight()
{
  MFDClass::updateTopic();
}

void OccupancyGridDisplay::updateMinHeight()
{
  MFDClass::updateTopic();
}

void OccupancyGridDisplay::clear()
{
  std::unique_lock<std::mutex> lock(mutex_);

  // reset rviz pointcloud boxes
  for (size_t i = 0; i < cloud_.size(); ++i) {
    cloud_[i]->clear();
  }
}

void OccupancyGridDisplay::update(
  [[maybe_unused]] float wall_dt, [[maybe_unused]] float ros_dt)
{
  if (new_points_received_) {
    std::unique_lock<std::mutex> lock(mutex_);

    for (size_t i = 0; i < MAX_OCTREE_DEPTH; ++i) {
      double size = box_size_[i];

      cloud_[i]->clear();
      cloud_[i]->setDimensions(size, size, size);

      cloud_[i]->addPoints(new_points_[i].begin(), new_points_[i].end());
      new_points_[i].clear();
      cloud_[i]->setAlpha(alpha_property_->getFloat());
    }
    new_points_received_ = false;
  }
  updateFromTF();
}

void OccupancyGridDisplay::reset()
{
  MFDClass::reset();
  clear();
  setStatusStd(StatusProperty::Ok, "Messages", "0 binary octomap messages received");
}

bool OccupancyGridDisplay::updateFromTF()
{
  // Check if frame exists to avoid spamming the RViz console output
  std::string error{};
  if (context_->getFrameManager()->transformHasProblems(header_.frame_id, error)) {
    return false;
  }

  // get tf transform
  Ogre::Vector3 pos;
  Ogre::Quaternion orient;
  if (!context_->getFrameManager()->getTransform(header_, pos, orient)) {
    return false;
  }

  scene_node_->setOrientation(orient);
  scene_node_->setPosition(pos);
  return true;
}

template<typename OcTreeType>
bool TemplatedOccupancyGridDisplay<OcTreeType>::checkType(std::string type_id)
{
  // General case: Need to be specialized for every used case
  setStatusStd(StatusProperty::Warn, "Messages", "Cannot verify octomap type");
  return true;  // Try deserialization, might crash though
}

template<>
bool TemplatedOccupancyGridDisplay<octomap::OcTreeStamped>::checkType(std::string type_id)
{
  if (type_id == "OcTreeStamped") {return true;} else {return false;}
}
template<>
bool TemplatedOccupancyGridDisplay<octomap::OcTree>::checkType(std::string type_id)
{
  if (type_id == "OcTree") {return true;} else {return false;}
}

template<>
bool TemplatedOccupancyGridDisplay<octomap::ColorOcTree>::checkType(std::string type_id)
{
  if (type_id == "ColorOcTree") {return true;} else {return false;}
}

template<typename OcTreeType>
void TemplatedOccupancyGridDisplay<OcTreeType>::setVoxelColor(
  rviz_rendering::PointCloud::Point & new_point,
  typename OcTreeType::NodeType & node,
  double min_z, double max_z)
{
  OctreeVoxelColorMode octree_color_mode =
    static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt());
  float cell_probability{};
  switch (octree_color_mode) {
    case OCTOMAP_CELL_COLOR:
      setStatusStd(StatusProperty::Error, "Messages", "Cannot extract color");
      break;
    // Intentional fall-through for else-case
    case OCTOMAP_Z_AXIS_COLOR:
      setColor(new_point.position.z, min_z, max_z, color_factor_, new_point);
      break;
    case OCTOMAP_PROBABILITY_COLOR:
      cell_probability = node.getOccupancy();
      new_point.setColor((1.0f - cell_probability), cell_probability, 0.0);
      break;
    default:
      break;
  }
}

// Specialization for ColorOcTreeNode, which can set the voxel color from the node itself
template<>
void TemplatedOccupancyGridDisplay<octomap::ColorOcTree>::setVoxelColor(
  rviz_rendering::PointCloud::Point & new_point,
  octomap::ColorOcTree::NodeType & node,
  double min_z, double max_z)
{
  float cell_probability{};
  OctreeVoxelColorMode octree_color_mode =
    static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt());
  switch (octree_color_mode) {
    case OCTOMAP_CELL_COLOR:
      {
        const float b2f = 1. / 256.;
        octomap::ColorOcTreeNode::Color & color = node.getColor();
        new_point.setColor(b2f * color.r, b2f * color.g, b2f * color.b, node.getOccupancy());
        break;
      }
    case OCTOMAP_Z_AXIS_COLOR:
      setColor(new_point.position.z, min_z, max_z, color_factor_, new_point);
      break;
    case OCTOMAP_PROBABILITY_COLOR:
      cell_probability = node.getOccupancy();
      new_point.setColor((1.0f - cell_probability), cell_probability, 0.0);
      break;
    default:
      break;
  }
}

template<typename OcTreeType>
void TemplatedOccupancyGridDisplay<OcTreeType>::processMessage(
  const octomap_msgs::msg::Octomap::ConstSharedPtr msg)
{
  setStatusStd(StatusProperty::Ok, "Type", msg->id.c_str());
  if (!checkType(msg->id)) {
    setStatusStd(
      StatusProperty::Error, "Message",
      "Wrong octomap type. Use a different display type.");
    return;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("rviz2"),
    "Received OctomapBinary message (size: %zu bytes)", msg->data.size()
  );

  header_ = msg->header;
  if (!updateFromTF()) {
    std::stringstream ss;
    ss << "Failed to transform from frame [" << header_.frame_id << "] to frame [" <<
      context_->getFrameManager()->getFixedFrame() << "]";
    setStatusStd(StatusProperty::Error, "Message", ss.str());
    return;
  }

  // creating octree
  OcTreeType * octomap = NULL;
  octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(*msg);
  if (tree) {
    octomap = dynamic_cast<OcTreeType *>(tree);
    if (!octomap) {
      setStatusStd(
        StatusProperty::Error, "Message",
        "Wrong octomap type. Use a different display type.");
    }
  } else {
    setStatusStd(StatusProperty::Error, "Message", "Failed to deserialize octree message.");
    return;
  }


  tree_depth_property_->setMax(octomap->getTreeDepth());

  // get dimensions of octree
  double min_x{};
  double min_y{};
  double min_z{};
  double max_x{};
  double max_y{};
  double max_z{};
  octomap->getMetricMin(min_x, min_y, min_z);
  octomap->getMetricMax(max_x, max_y, max_z);

  // reset rviz pointcloud classes
  for (std::size_t i = 0; i < MAX_OCTREE_DEPTH; ++i) {
    point_buf_[i].clear();
    box_size_[i] = octomap->getNodeSize(i + 1);
  }

  size_t point_count = 0;
  {
    // traverse all leafs in the tree:
    unsigned int tree_depth = std::min<unsigned int>(
      tree_depth_property_->getInt(), octomap->getTreeDepth());
    double max_height = std::min<double>(max_height_property_->getFloat(), max_z);
    double min_height = std::max<double>(min_height_property_->getFloat(), min_z);
    int step_size = 1 << (octomap->getTreeDepth() - tree_depth);  // for pruning of occluded voxels
    for (typename OcTreeType::iterator it = octomap->begin(tree_depth), end = octomap->end();
      it != end; ++it)
    {
      if (it.getZ() <= max_height && it.getZ() >= min_height) {
        int render_mode_mask = octree_render_property_->getOptionInt();

        bool display_voxel = false;

        // the left part evaluates to 1 for free voxels and 2 for occupied voxels
        if ((static_cast<int>(octomap->isNodeOccupied(*it)) + 1) & render_mode_mask) {
          // check if current voxel has neighbors on all sides -> no need to be displayed
          bool all_neighbors_found = true;

          octomap::OcTreeKey key;
          octomap::OcTreeKey n_key = it.getKey();

          // determine indices of potentially neighboring voxels for depths < maximum tree depth
          // +/-1 at maximum depth, +2^(depth_difference-1)
          // and -2^(depth_difference-1)-1 on other depths
          int diff_base =
            (it.getDepth() <
            octomap->getTreeDepth()) ? 1 << (octomap->getTreeDepth() - it.getDepth() - 1) : 1;
          int diff[2] =
          {-((it.getDepth() == octomap->getTreeDepth()) ? diff_base : diff_base + 1), diff_base};

          // cells with adjacent faces can occlude a voxel,
          // iterate over the cases x,y,z (idx_case) and +/- (diff)
          for (unsigned int idx_case = 0; idx_case < 3; ++idx_case) {
            int idx_0 = idx_case % 3;
            int idx_1 = (idx_case + 1) % 3;
            int idx_2 = (idx_case + 2) % 3;

            for (int i = 0; all_neighbors_found && i < 2; ++i) {
              key[idx_0] = n_key[idx_0] + diff[i];
              // if rendering is restricted to tree_depth < maximum tree depth inner nodes
              // with distance step_size can already occlude a voxel
              for (key[idx_1] = n_key[idx_1] + diff[0] + 1;
                all_neighbors_found && key[idx_1] < n_key[idx_1] + diff[1]; key[idx_1] += step_size)
              {
                for (key[idx_2] = n_key[idx_2] + diff[0] + 1;
                  all_neighbors_found && key[idx_2] < n_key[idx_2] + diff[1];
                  key[idx_2] += step_size)
                {
                  typename OcTreeType::NodeType * node = octomap->search(key, tree_depth);

                  // the left part evaluates to 1 for free voxels and 2 for occupied voxels
                  if (!(node &&
                    ((static_cast<int>(octomap->isNodeOccupied(node)) + 1) & render_mode_mask)))
                  {
                    // we do not have a neighbor => break!
                    all_neighbors_found = false;
                  }
                }
              }
            }
          }

          display_voxel |= !all_neighbors_found;
        }


        if (display_voxel) {
          rviz_rendering::PointCloud::Point new_point;

          new_point.position.x = it.getX();
          new_point.position.y = it.getY();
          new_point.position.z = it.getZ();


          setVoxelColor(new_point, *it, min_z, max_z);
          // push to point vectors
          unsigned int depth = it.getDepth();
          point_buf_[depth - 1].push_back(new_point);

          ++point_count;
        }
      }
    }
  }

  if (point_count) {
    std::unique_lock<std::mutex> lock(mutex_);

    new_points_received_ = true;

    for (size_t i = 0; i < MAX_OCTREE_DEPTH; ++i) {
      new_points_[i].swap(point_buf_[i]);
    }
  }
  delete octomap;
}

using OcTreeGridDisplay = TemplatedOccupancyGridDisplay<octomap::OcTree>;
using ColorOcTreeGridDisplay = TemplatedOccupancyGridDisplay<octomap::ColorOcTree>;
using OcTreeStampedGridDisplay = TemplatedOccupancyGridDisplay<octomap::OcTreeStamped>;
}  // namespace octomap_rviz_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(octomap_rviz_plugins::OcTreeGridDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(octomap_rviz_plugins::ColorOcTreeGridDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(octomap_rviz_plugins::OcTreeStampedGridDisplay, rviz_common::Display)
