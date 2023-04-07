#pragma once
#include <ros/ros.h>
#include <oakd_msgs/TrackletArray.h>
#include <oakd_msgs/Tracklet.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <string>
#include "common.hpp"

namespace oakd
{
    class MarkerPublisher
    {
    private:
        ros::NodeHandle nh_;
        std::string node_name_;
        ros::Publisher pub_marker_;
        ros::Subscriber sub_tracklets_;

        void cbTrackletMsg(const oakd_msgs::TrackletArrayConstPtr &msg)
        {
            visualization_msgs::MarkerArray markers;
            for (auto &track : msg->tracklets)
            {
                visualization_msgs::Marker line_strip;
                line_strip.header.frame_id = "oak-d-base-frame";
                line_strip.header.stamp = ros::Time::now();
                line_strip.ns = track.label;
                line_strip.id = track.id;
                line_strip.action = visualization_msgs::Marker::ADD;
                line_strip.pose.orientation.w = 1.0;
                line_strip.type = visualization_msgs::Marker::LINE_STRIP;
                line_strip.scale.x = 0.1;
                line_strip.color.r = 1.0;
                line_strip.color.a = 1.0;

                auto width = track.roi.size_x / 100.0;
                auto height = track.roi.size_y / 100.0;
                
                // center
                geometry_msgs::Point center;
                center.x = track.spatialCoordinates.x / 1000.0;
                center.y = track.spatialCoordinates.y / 1000.0;
                center.z = track.spatialCoordinates.z / 1000.0;
                // topLeft
                geometry_msgs::Point topLeft;
                topLeft.x = center.x - width / 2.0;
                topLeft.y = center.y - height / 2.0;
                topLeft.z = center.z;
                // topRight
                geometry_msgs::Point topRight;
                topRight.x = center.x + width / 2.0;
                topRight.y = center.y - height / 2.0;
                topRight.z = center.z;
                // bottomLeft
                geometry_msgs::Point bottomLeft;
                bottomLeft.x = center.x - width / 2.0;
                bottomLeft.y = center.y + height / 2.0;
                bottomLeft.z = center.z;
                // bottomRight
                geometry_msgs::Point bottomRight;
                bottomRight.x = center.x + width / 2.0;
                bottomRight.y = center.y + height / 2.0;
                bottomRight.z = center.z;

                line_strip.points.push_back(topLeft);
                line_strip.points.push_back(topRight);
                line_strip.points.push_back(bottomRight);
                line_strip.points.push_back(bottomLeft);
                line_strip.points.push_back(topLeft);

                line_strip.lifetime = ros::Duration(0.25);

                markers.markers.push_back(line_strip);
            }
            pub_marker_.publish(markers);
        }

    public:
        MarkerPublisher(ros::NodeHandle nh, std::string node_name) : nh_{nh},
                                                                     node_name_{node_name}
        {
            ROS_INFO_STREAM("Init " + node_name_);
            sub_tracklets_ = nh_.subscribe(oakd::TRACKLET_TOPIC, 1, &MarkerPublisher::cbTrackletMsg, this);
            pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(oakd::MARKER_TOPIC, 1);
        }

        ~MarkerPublisher() = default;
    };
};