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
        double c_x_, c_y_, f_x_, f_y_;
        std::string frame_id_;

        inline geometry_msgs::Point get3DPoint(double u, double v, double distance)
        {
            geometry_msgs::Point pt_3d;
            pt_3d.x = ((u - c_x_) * distance) / f_x_;
            pt_3d.y = ((v - c_y_) * distance) / f_y_;
            pt_3d.z = distance;
            return pt_3d;
        }

        inline std::tuple<geometry_msgs::Point, double, double, visualization_msgs::Marker> getBoundingBox(const std::string label,
                                                                                                           const int id,
                                                                                                           const double &distance,
                                                                                                           const int width_px,
                                                                                                           const int height_px,
                                                                                                           const cv::Point &center_px)
        {
            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = frame_id_;
            line_strip.header.stamp = ros::Time::now();
            line_strip.ns = label;
            line_strip.id = id;
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.pose.orientation.w = 1.0;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x = 0.01;
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;
            line_strip.lifetime = ros::Duration(0.25);
            std::vector<geometry_msgs::Point> points_3d;
            float a{1.0}, b{1.0};
            for (size_t i = 1; i <= 5; ++i)
            {
                auto u = center_px.x + a * width_px / 2.0;
                auto v = center_px.y + b * height_px / 2.0;
                auto pt_3d = get3DPoint(u, v, distance);
                points_3d.push_back(pt_3d);
                if (i == 1)
                    a = -1.0;
                else if (i == 2)
                    b = -1.0;
                else if (i == 3)
                    a = 1.0;
                else if (i == 4)
                    b = 1.0;
            }
            line_strip.points = points_3d;

            auto width = points_3d.at(0).x - points_3d.at(1).x;
            auto height = points_3d.at(1).y - points_3d.at(2).y;
            auto center = get3DPoint(center_px.x, center_px.y, distance);

            return {center, width, height, line_strip};
        }

        inline visualization_msgs::Marker getText(const std::string &label,
                                                  const int id,
                                                  const double &distance,
                                                  const double width,
                                                  const double height,
                                                  const geometry_msgs::Point &center,
                                                  const double &confidence,
                                                  const int status)
        {
            visualization_msgs::Marker text;
            text.header.frame_id = frame_id_;
            text.header.stamp = ros::Time::now();
            text.ns = label;
            text.id = -id - 1;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.orientation.w = 1.0;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.scale.z = 0.2;
            text.color.r = 1.0;
            text.color.a = 1.0;
            text.lifetime = ros::Duration(0.25);
            text.pose.position = center;
            std::stringstream sstream;
            sstream << "label: " << label << "\nid: " << id << "\nstatus: " << status << "\nconfidence: " << confidence <<
                        "\nheight: " << height << "\nwidth: " << width << "\ndistance: " << distance;
            text.text = sstream.str();
            return text;
        }

        void cbTrackletAndCameraInfo(const oakd_msgs::TrackletArrayConstPtr &tracklet_msg,
                                     const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
        {
            image_geometry::PinholeCameraModel camera_model;
            camera_model.fromCameraInfo(camera_info_msg);
            c_x_ = camera_model.cx();
            c_y_ = camera_model.cy();
            f_x_ = camera_model.fx();
            f_y_ = camera_model.fy();
            frame_id_ = camera_model.tfFrame();

            visualization_msgs::MarkerArray markers;
            for (auto &track : tracklet_msg->tracklets)
            {
                auto dist = track.spatialCoordinates.z / 1000.0; 
                // bbox
                auto width_px = track.roi.size_x;
                auto height_px = track.roi.size_y;
                // detection confidence
                auto confidence = track.srcImgDetection.confidence;
                // center in pixel
                cv::Point center_px;
                center_px.x = track.roi.center.x;
                center_px.y = track.roi.center.y;
                // detection label
                auto label = labelMap[track.label];
                // tracking status
                auto status = track.status;
                // fill marker array
                auto [center, width, height, bbox] = getBoundingBox(label, track.id, dist, width_px, height_px, center_px);
                markers.markers.push_back(bbox);
                auto text = getText(label, track.id, dist, width, height, center, confidence, status);
                markers.markers.push_back(text);
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