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

#include "octomap_rviz_plugins/octomap_server_control.h"

namespace octomap_rviz_plugin {
OctomapServerControl::OctomapServerControl(QWidget* parent)
    : rviz::Panel(parent) {
  vbox_ = new QVBoxLayout();

  btnResetOctomap_ = new QPushButton(tr("Reset Octomap"));
  btnResetOctomap_->setSizePolicy(QSizePolicy::Expanding,
                                  QSizePolicy::Expanding);

  vbox_->addWidget(btnResetOctomap_);

  connect(btnResetOctomap_, SIGNAL(clicked()), this, SLOT(resetOctomap()));

  setLayout(vbox_);
}

void OctomapServerControl::resetOctomap() {
  std_srvs::Empty req;
  srvResetOctomap_.call(req);
}
}

#include <pluginlib/class_list_macros.h>

typedef octomap_rviz_plugin::OctomapServerControl OctomapServerControl;
PLUGINLIB_EXPORT_CLASS(OctomapServerControl, rviz::Panel)
