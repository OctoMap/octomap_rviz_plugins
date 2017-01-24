/*
 * Copyright (c) 2017, Wolfgang Merkt
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
 *
 * Author: Wolfgang Merkt (wolfgang.merkt@ed.ac.uk)
 *
 */

#ifndef RVIZ_OCTOMAP_SERVER_CONTROL_H
#define RVIZ_OCTOMAP_SERVER_CONTROL_H

#ifndef Q_MOC_RUN

#include <QtGui>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_srvs/Empty.h>

#endif

namespace octomap_rviz_plugin {

class OctomapServerControl : public rviz::Panel {
  Q_OBJECT

 public:
  OctomapServerControl(QWidget* parent = 0);

 protected Q_SLOTS:
  void resetOctomap();

 protected:
  QVBoxLayout* vbox_;
  QPushButton* btnResetOctomap_;

 private:
  ros::NodeHandle nh_;
  ros::ServiceClient srvResetOctomap_ =
      nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");
};
}

#endif
