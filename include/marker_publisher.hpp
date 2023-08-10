#pragma once
#include <ros/ros.h>
#include <depthai_ros_msgs/TrackletArray.h>
#include <depthai_ros_msgs/Tracklet.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <depthai/depthai.hpp>

#include "common.hpp"
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <tf/tf.h>
#include <cmath>

namespace oakd
{
    struct Marker
    {
        int label;
        int status;
        double distance;
        double width;
        double height;
    };

    class MarkerPublisher
    {
    private:
        ros::NodeHandle nh_;
        std::map<int, std::vector<Marker>> markers_;
        std::vector<std::vector<double>> ekf_info_;
        std::string node_name_;
        message_filters::Subscriber<depthai_ros_msgs::TrackletArray> sub_tracklets_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
        typedef message_filters::sync_policies::ApproximateTime<depthai_ros_msgs::TrackletArray, sensor_msgs::CameraInfo> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        ros::Publisher pub_marker_;
        ros::Subscriber sub_odom_;
        nav_msgs::Odometry odom_;
        nav_msgs::Odometry prev_odom_;
        double c_x_, c_y_, f_x_, f_y_;
        std::string frame_id_;

        inline geometry_msgs::Point get3DPoint(double u, double v, double distance)
        {
            geometry_msgs::Point pt_3d;
            pt_3d.x = ((u - c_x_) * distance) / f_x_;
            pt_3d.y = ((v - c_y_) * distance) / f_y_;
            pt_3d.z = distance;
            // ROS_INFO_STREAM(distance);
            return pt_3d;
        }

        void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
        {
            odom_ = *msg;
        }

        inline std::tuple<geometry_msgs::Point, double, double, visualization_msgs::Marker> getBoundingBox(const std::string label,
                                                                                                           const int id,
                                                                                                           const double &distance,
                                                                                                           const int width_px,
                                                                                                           const int height_px,
                                                                                                           const cv::Point &center_px,
                                                                                                           float blue,
                                                                                                           float green,
                                                                                                           float red)
        {
            visualization_msgs::Marker cube;
            cube.header.frame_id = frame_id_;
            cube.header.stamp = ros::Time::now();
            cube.ns = label;
            cube.id = id;
            cube.action = visualization_msgs::Marker::ADD;
            cube.pose.orientation.w = 1.0;
            cube.type = visualization_msgs::Marker::CUBE;
            cube.color.r = red;
            cube.color.b = blue;
            cube.color.g = green;
            cube.color.a = 0.3;
            cube.lifetime = ros::Duration(2000);
            std::vector<geometry_msgs::Point> points_3d;

            float a{1.0}, b{1.0};
            for (size_t i = 1; i <= 4; ++i)
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
            }

            auto width = points_3d.at(0).x - points_3d.at(1).x;
            auto height = points_3d.at(1).y - points_3d.at(2).y;
            auto depth = width < height ? width : height;
            cube.scale.x = width;
            cube.scale.y = height;
            cube.scale.z = depth;

            auto center = get3DPoint(center_px.x, center_px.y, distance);
            cube.pose.position = center;

            return {center, width, height, cube};
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
            text.scale.z = 0.02;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;
            text.lifetime = ros::Duration(2000);
            auto position = center;
            position.y -= 0.02;
            text.pose.position = position;
            std::stringstream sstream;
            auto area = height * width;
            // sstream << "label: " << label << "\nid: " << id << "\nstatus: " << status << "\nconfidence: " << confidence << "\nheight: " << height << "\nwidth: " << width << "\ndistance: " << distance;
            // sstream << "label: " << label << "\nheight: " << height << "\nwidth: " << width << "\ndistance: " << distance;
            sstream << id;
            // sstream << area;
            text.text = sstream.str();
            return text;
        }

        void cbTrackletAndCameraInfo(const depthai_ros_msgs::TrackletArrayConstPtr &tracklet_msg,
                                     const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
        {
            image_geometry::PinholeCameraModel camera_model;
            camera_model.fromCameraInfo(camera_info_msg);
            c_x_ = camera_model.cx();
            c_y_ = camera_model.cy();
            f_x_ = camera_model.fx();
            f_y_ = camera_model.fy();
            frame_id_ = camera_model.tfFrame();

            tf::Pose pose;
            tf::poseMsgToTF(odom_.pose.pose, pose);
            double robot_yaw = tf::getYaw(pose.getRotation());
            auto robot_x = odom_.pose.pose.position.x;
            auto robot_y = odom_.pose.pose.position.y;
            auto robot_v = odom_.twist.twist.linear.x;
            auto robot_yaw_rate = odom_.twist.twist.angular.z;

            std::vector info{robot_x, robot_y, robot_yaw, robot_v, robot_yaw_rate};

            visualization_msgs::MarkerArray markers;
            for (auto &track : tracklet_msg->tracklets)
            {
                // id, label, status, dist, width, height

                // "Flower" = 0
                // "Strawberry" = 1
                // "StrawberryNotReady" = 2

                // NEW = 0
                // TRACKED = 1
                // LOST = 2
                // REMOVED = 3

                if (track.status == (int)dai::Tracklet::TrackingStatus::TRACKED && track.spatialCoordinates.z > 0.01 && track.spatialCoordinates.z < 1.0)
                {
                    auto dist = track.spatialCoordinates.z;
                    // bbox
                    auto width_px = track.roi.size_x;
                    auto height_px = track.roi.size_y;
                    // detection confidence
                    auto confidence = track.srcImgDetectionHypothesis.score;
                    // center in pixel
                    cv::Point center_px;
                    center_px.x = track.roi.center.x;
                    center_px.y = track.roi.center.y;
                    // detection label
                    auto label = labelMap[track.label];
                    // tracking status
                    auto status = track.status;
                    // fill marker array
                    float b{1.0}, g{1.0}, r{1.0};
                    if (track.label == 1)
                        b = g = 0.0;
                    else if (track.label == 2)
                        b = r = 0.0;
                    geometry_msgs::Point center;
                    double width, height;
                    visualization_msgs::Marker bbox;
                    std::tie(center, width, height, bbox) = getBoundingBox(label, track.id, dist, width_px, height_px, center_px, b, g, r);
                    markers.markers.push_back(bbox);

                    markers_[track.id].push_back({track.label, track.status, dist, width, height});

                    std::map<int, int> keys;
                    int i{1};
                    for (const auto &[key, values] : markers_)
                    {
                        keys[key] = i;
                        i++;
                    }

                    auto lm_x = center.x * cos(robot_yaw) - center.z * sin(robot_yaw) + robot_x;
                    auto lm_y = center.x * sin(robot_yaw) + center.z * cos(robot_yaw) + robot_y;

                    info.push_back(keys[track.id] - 1);
                    info.push_back(lm_x);
                    info.push_back(lm_y);

                    auto text = getText(label, keys[track.id], dist, width, height, center, confidence, status);
                    markers.markers.push_back(text);
                }
            }
            ekf_info_.push_back(info);
            prev_odom_ = odom_;

            pub_marker_.publish(markers);
        }

        void saveData()
        {
            std::ofstream MyFile("/home/nino/output.csv");
            for (const auto &[key, values] : markers_)
            {
                MyFile << key;
                for (const auto &value : values)
                {
                    MyFile << "," << value.label << "," << value.status << "," << value.distance << "," << value.width << "," << value.height;
                }
                MyFile << "\n";
            }
            MyFile.close();

            std::ofstream MyOtherFile("/home/nino/ekf.csv");
            for (const auto &info : ekf_info_)
            {
                for (const auto &entry : info)
                {
                    MyOtherFile << entry << ",";
                }
                MyOtherFile << "\n";
            }
            MyOtherFile.close();
        }

    public:
        MarkerPublisher(ros::NodeHandle nh, std::string node_name) : nh_{nh},
                                                                     node_name_{node_name},
                                                                     prev_odom_{odom_}
        {
            ROS_INFO_STREAM("Init " + node_name_);
            sub_tracklets_.subscribe(nh_, oakd::TRACKLET_TOPIC, 1);
            sub_camera_info_.subscribe(nh_, oakd::CAMERA_INFO_TOPIC, 1);
            sync_.reset(new Sync(MySyncPolicy(10), sub_tracklets_, sub_camera_info_));
            sync_->registerCallback(boost::bind(&MarkerPublisher::cbTrackletAndCameraInfo, this, _1, _2));
            pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(oakd::MARKER_TOPIC, 1);
            sub_odom_ = nh_.subscribe("/odom", 1, &MarkerPublisher::cbOdom, this);
        }

        ~MarkerPublisher()
        {
            saveData();
        };
    };
};