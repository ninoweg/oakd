#pragma once
#include <ros/ros.h>
#include <oakd_msgs/TrackletArray.h>
#include <oakd_msgs/Tracklet.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include "common.hpp"

namespace oakd
{
    class MarkerPublisher
    {
    private:
        ros::NodeHandle nh_;
        std::string node_name_;
        message_filters::Subscriber<oakd_msgs::TrackletArray> sub_tracklets_;  
        message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;               
        typedef message_filters::sync_policies::ApproximateTime<oakd_msgs::TrackletArray, sensor_msgs::CameraInfo> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        ros::Publisher pub_marker_;
        image_geometry::PinholeCameraModel camera_model_;

        void cbTrackletAndCameraInfo(const oakd_msgs::TrackletArrayConstPtr &tracklet_msg, const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
        {
            camera_model_.fromCameraInfo(camera_info_msg);
            auto c_x = camera_model_.cx();
            auto c_y = camera_model_.cy();
            auto f_x = camera_model_.fx();
            auto f_y = camera_model_.fy();
            
            visualization_msgs::MarkerArray markers;
            for (auto &track : tracklet_msg->tracklets)
            {
                visualization_msgs::Marker line_strip;
                line_strip.header.frame_id = camera_model_.tfFrame();
                line_strip.header.stamp = ros::Time::now();
                line_strip.ns = track.label;
                line_strip.id = track.id;
                line_strip.action = visualization_msgs::Marker::ADD;
                line_strip.pose.orientation.w = 1.0;
                line_strip.type = visualization_msgs::Marker::LINE_STRIP;
                line_strip.scale.x = 0.1;
                line_strip.color.r = 1.0;
                line_strip.color.a = 1.0;

                auto width = track.roi.size_x;
                auto height = track.roi.size_y;
                auto dist = track.spatialCoordinates.z / 1000.0;

                std::vector<cv::Point2d> points_2d;
                // center
                cv::Point2d center;
                center.x = track.spatialCoordinates.x / 1000.0;
                center.y = track.spatialCoordinates.y / 1000.0;
                // topLeft
                cv::Point2d top_left;
                top_left.x = center.x - width / 2.0;
                top_left.y = center.y - height / 2.0;
                points_2d.push_back(top_left);
                // topRight
                cv::Point2d top_right;
                top_right.x = center.x + width / 2.0;
                top_right.y = center.y - height / 2.0;
                points_2d.push_back(top_right);
                // bottomRight
                cv::Point2d bottom_right;
                bottom_right.x = center.x + width / 2.0;
                bottom_right.y = center.y + height / 2.0;
                points_2d.push_back(bottom_right);
                // bottomLeft
                cv::Point2d bottom_left;
                bottom_left.x = center.x - width / 2.0;
                bottom_left.y = center.y + height / 2.0;
                points_2d.push_back(bottom_left);

                points_2d.push_back(top_left);

                std::vector<geometry_msgs::Point> points_3d;
                for (auto& pt_2d : points_2d)
                {
                    // cv::Point3d pt_cv = camera_model_.projectPixelTo3dRay(pt_2d) * dist;
                    geometry_msgs::Point pt_3d;
                    auto u = pt_2d.x;
                    auto v = pt_2d.y;
                    
                    pt_3d.x = dist;
                    pt_3d.y = ((u - c_x) * dist) / (f_x);
                    pt_3d.z = ((v - c_y) * dist) / (f_y);
                    
                    points_3d.push_back(pt_3d);
                }

                line_strip.points = points_3d;
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
            sub_tracklets_.subscribe(nh_, oakd::TRACKLET_TOPIC, 1);
            sub_camera_info_.subscribe(nh_, oakd::CAMERA_INFO_TOPIC, 1);
            sync_.reset(new Sync(MySyncPolicy(10), sub_tracklets_, sub_camera_info_));
            sync_->registerCallback(boost::bind(&MarkerPublisher::cbTrackletAndCameraInfo, this, _1, _2));
            pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(oakd::MARKER_TOPIC, 1);
        }

        ~MarkerPublisher() = default;
    };
};