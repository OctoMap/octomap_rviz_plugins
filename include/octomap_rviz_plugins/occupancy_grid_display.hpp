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

#ifndef OCTOMAP_RVIZ_PLUGINS__OCCUPANCY_GRID_DISPLAY_HPP_
#define OCTOMAP_RVIZ_PLUGINS__OCCUPANCY_GRID_DISPLAY_HPP_

#ifndef Q_MOC_RUN

#include <memory>
#include <string>
#include <vector>

#include "octomap_msgs/msg/octomap.hpp"

#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_common/message_filter_display.hpp"

#endif

namespace rviz_common
{
namespace properties
{
class IntProperty;
class EnumProperty;
class FloatProperty;
}
}

namespace octomap_rviz_plugins
{
static constexpr std::size_t MAX_OCTREE_DEPTH = sizeof(uint16_t) * 8;

class OccupancyGridDisplay
  : public rviz_common::MessageFilterDisplay<octomap_msgs::msg::Octomap>
{
  Q_OBJECT

public:
  OccupancyGridDisplay();

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private Q_SLOTS:
  void updateTreeDepth();
  void updateOctreeRenderMode();
  void updateOctreeColorMode();
  void updateAlpha();
  void updateMaxHeight();
  void updateMinHeight();

protected:
  void unsubscribe() override;

  void setColor(
    double z_pos, double min_z, double max_z, double color_factor,
    rviz_rendering::PointCloud::Point & point);

  void clear();

  virtual bool updateFromTF();

  using VPoint = std::vector<rviz_rendering::PointCloud::Point>;
  using VVPoint = std::vector<VPoint>;

  std::mutex mutex_;

  // point buffer
  VVPoint new_points_;
  VVPoint point_buf_;
  bool new_points_received_{false};

  // Ogre-rviz point clouds
  std::vector<std::shared_ptr<rviz_rendering::PointCloud>> cloud_;
  std::vector<double> box_size_;
  std_msgs::msg::Header header_;

  // Plugin properties
  rviz_common::properties::EnumProperty * octree_render_property_;
  rviz_common::properties::EnumProperty * octree_coloring_property_;
  rviz_common::properties::IntProperty * tree_depth_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * max_height_property_;
  rviz_common::properties::FloatProperty * min_height_property_;

  double color_factor_{0.8};
};

template<typename OcTreeType>
class TemplatedOccupancyGridDisplay : public OccupancyGridDisplay
{
protected:
  void processMessage(const octomap_msgs::msg::Octomap::ConstSharedPtr msg) override;
  void setVoxelColor(
    rviz_rendering::PointCloud::Point & new_point,
    typename OcTreeType::NodeType & node, double min_z, double max_z);
  /// Returns false, if the type_id (of the message) does not correspond to the template paramter
  /// of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);
};

}  // namespace octomap_rviz_plugins

#endif  // OCTOMAP_RVIZ_PLUGINS__OCCUPANCY_GRID_DISPLAY_HPP_
