/*
 Software License Agreement (BSD License)

 Copyright (c) 2012, Scott Niekum
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

 author: Scott Niekum
*/
#include "IndividualMarkersNoKinectNode.h"

void IndividualMarkersNoKinectNode::cameraCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    //If we've already gotten the cam info, then go ahead
    if(cam->getCamInfo_){
        try{

            //Convert the image
            cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

            //Get the estimated pose of the main markers by using all the markers in each bundle

            // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
            // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
            // do this conversion here -jbinney
            IplImage ipl_image = cv_ptr_->image;

            marker_detector->Detect(&ipl_image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true);

            arPoseMarkers_.markers.clear ();
            for (size_t i=0; i<marker_detector->markers->size(); i++)
            {
                //Get the pose relative to the camera
                int id = (*(marker_detector->markers))[i].GetId();
                Pose p = (*(marker_detector->markers))[i].pose;
                //Create the pose marker messages
                ar_track_alvar_msgs::AlvarMarker ar_pose_marker;

                ar_pose_marker.pose.pose.position.x = p.translation[0]/100.0;
                ar_pose_marker.pose.pose.position.y = p.translation[1]/100.0;
                ar_pose_marker.pose.pose.position.z = p.translation[2]/100.0;
                ar_pose_marker.pose.pose.orientation.x = p.quaternion[0];
                ar_pose_marker.pose.pose.orientation.y = p.quaternion[1];
                ar_pose_marker.pose.pose.orientation.z = p.quaternion[2];
                ar_pose_marker.pose.pose.orientation.w = p.quaternion[3];

                ar_pose_marker.header.stamp = image_msg->header.stamp;
                ar_pose_marker.id = id;
                arPoseMarkers_.markers.push_back (ar_pose_marker);

            }
            arMarkerPub_.publish (arPoseMarkers_);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
        }
    }
}

IndividualMarkersNoKinectNode::IndividualMarkersNoKinectNode(ros::NodeHandle _nh, ros::NodeHandle private_nh):
    node(_nh), pr_node(private_nh), it(_nh)
{

    pr_node.getParam("marker_size", marker_size);
    pr_node.getParam("max_new_marker_error", max_new_marker_error);
    pr_node.getParam("max_track_error", max_track_error);
    pr_node.getParam("cam_image_topic", cam_image_topic);
    pr_node.getParam("cam_info_topic", cam_info_topic);

    cam = new Camera(node, cam_info_topic);
    marker_detector = new MarkerDetector<MarkerData>();
    arMarkerPub_ = node.advertise<ar_track_alvar_msgs::AlvarMarkers> ("ar_pose_marker", 0);

    marker_detector->SetMarkerSize(marker_size);

    ROS_INFO("Try subscribe to %s", cam_image_topic.c_str() );
    cam_sub_ = it.subscribe(cam_image_topic, 1, &IndividualMarkersNoKinectNode::cameraCallback, this);

}

IndividualMarkersNoKinectNode::~IndividualMarkersNoKinectNode()
{
    if (cam) delete cam;
    if (marker_detector) delete marker_detector;
}





