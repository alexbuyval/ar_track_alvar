/*
 Software License Agreement (BSD License)

 Copyright (c) 2015, Alex Buyval
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the Willow Garage nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 author: Alex Buyval
*/

#include "IndividualMarkersNoKinectNode.h"

#include <ros/ros.h>
#include <ros/time.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace alvar {


class ARTrackNodelet : public nodelet::Nodelet {
public:
  ARTrackNodelet() : running_(false) {}
  ~ARTrackNodelet();

private:
  virtual void onInit();

  volatile bool running_;
  IndividualMarkersNoKinectNode* node_;
};

ARTrackNodelet::~ARTrackNodelet() {
  if (node_) delete node_;
}

void ARTrackNodelet::onInit() {
  ros::NodeHandle nh(getNodeHandle());
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  node_= new IndividualMarkersNoKinectNode(nh, priv_nh);

}

};
// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(ar_track_alvar, ARTrackNodelet, alvar::ARTrackNodelet, nodelet::Nodelet);
//PLUGINLIB_EXPORT_CLASS(ARTrackNodelet,  nodelet::Nodelet);
