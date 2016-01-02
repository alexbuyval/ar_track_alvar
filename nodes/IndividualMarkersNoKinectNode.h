#pragma once

#ifndef __IndividualMarkersNoKinectNode_H
#define __IndividualMarkersNoKinectNode_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
//#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//#include <dynamic_reconfigure/server.h>
//#include <ar_track_alvar/ParamsConfig.h>

using namespace alvar;
using namespace std;

class IndividualMarkersNoKinectNode
{
private:

    Camera *cam;
    cv_bridge::CvImagePtr cv_ptr_;
    image_transport::Subscriber cam_sub_;
    ros::Publisher arMarkerPub_;
    ar_track_alvar_msgs::AlvarMarkers arPoseMarkers_;
    MarkerDetector<MarkerData>* marker_detector;

    double max_frequency;
    double marker_size;
    double max_new_marker_error;
    double max_track_error;
    std::string cam_image_topic;
    std::string cam_info_topic;
  
    ros::NodeHandle node, pr_node;
    image_transport::ImageTransport it;

public:

    IndividualMarkersNoKinectNode(ros::NodeHandle _nh, ros::NodeHandle private_nh);
    ~IndividualMarkersNoKinectNode();

    void cameraCallback(const sensor_msgs::ImageConstPtr &image_msg);

};

#endif

