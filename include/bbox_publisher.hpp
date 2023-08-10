#pragma once
#include <ros/ros.h>
#include <depthai_ros_msgs/TrackletArray.h>
#include <depthai_ros_msgs/Tracklet.h>
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
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <depthai/depthai.hpp>

#include "common.hpp"

namespace oakd
{
    struct Marker
    {
        int label;
        int status;
    };

    class BBoxPublisher
    {
    private:
        ros::NodeHandle nh_;
        std::string node_name_;
        std::vector<int> counter_;
        std::map<int, std::vector<Marker>> markers_;
        message_filters::Subscriber<depthai_ros_msgs::TrackletArray> sub_tracklets_;
        message_filters::Subscriber<sensor_msgs::Image> sub_detection_image_;
        message_filters::Subscriber<sensor_msgs::Image> sub_tracker_image_;
        typedef message_filters::sync_policies::ApproximateTime<depthai_ros_msgs::TrackletArray, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        image_transport::Publisher pub_bbox_detection_;
        image_transport::Publisher pub_bbox_tracker_;
        image_transport::ImageTransport it_;

        void cbSyncronizer(const depthai_ros_msgs::TrackletArrayConstPtr &tracklet_msg,
                           const sensor_msgs::ImageConstPtr &detection_image_msg,
                           const sensor_msgs::ImageConstPtr &tracker_image_msg)
        {
            cv_bridge::CvImagePtr cv_detection_ptr, cv_tracker_ptr;
            try
            {
                cv_detection_ptr = cv_bridge::toCvCopy(detection_image_msg, sensor_msgs::image_encodings::BGR8);
                cv_tracker_ptr = cv_bridge::toCvCopy(tracker_image_msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            cv::Mat detection_frame = cv_detection_ptr->image;
            cv::Mat tracker_frame = cv_tracker_ptr->image;
            auto trackletsData = tracklet_msg->tracklets;
            auto color = cv::Scalar(255, 255, 255);
                        
            for (auto &t : trackletsData)
            {
                auto tracker_center_x = t.roi.center.x;
                auto tracker_center_y = t.roi.center.y;
                auto tracker_size_x = t.roi.size_x;
                auto tracker_size_y = t.roi.size_y;

                auto detection_center_x = t.srcImgDetectionBBox.center.x;
                auto detection_center_y = t.srcImgDetectionBBox.center.y;
                auto detection_size_x = t.srcImgDetectionBBox.size_x;
                auto detection_size_y = t.srcImgDetectionBBox.size_y;

                int detection_x1 = detection_center_x - detection_size_x / 2.0;
                int detection_y1 = detection_center_y - detection_size_y / 2.0;
                int detection_x2 = detection_center_x + detection_size_x / 2.0;
                int detection_y2 = detection_center_y + detection_size_y / 2.0;

                int tracker_x1 = tracker_center_x - tracker_size_x / 2.0;
                int tracker_y1 = tracker_center_y - tracker_size_y / 2.0;
                int tracker_x2 = tracker_center_x + tracker_size_x / 2.0;
                int tracker_y2 = tracker_center_y + tracker_size_y / 2.0;

                uint32_t labelIndex = t.label;
                std::string labelStr = std::to_string(labelIndex);

                if (labelIndex < labelMap.size())
                    labelStr = labelMap[labelIndex];

                switch (labelIndex)
                {
                case 0:
                    color = cv::Scalar(255, 255, 255);
                    break;
                case 1:
                    color = cv::Scalar(0, 0, 255);
                    break;
                case 2:
                    color = cv::Scalar(0, 255, 0);
                    break;
                }

                // NEW TRACKLED LOST REMOVED
                
                if (t.status == (int)dai::Tracklet::TrackingStatus::TRACKED && t.spatialCoordinates.z > 0.01 && t.spatialCoordinates.z < 1.0)
                {
                    
                    markers_[t.id].push_back({t.label, t.status});

                    std::map<int, int> keys;
                    int i{1};
                    counter_ = {0,0,0};
                    for (const auto &[key, values] : markers_)
                    {
                        keys[key] = i;
                        counter_.at(values.at(0).label)++;
                        i++;
                    }

                    std::stringstream idStr;
                    idStr << keys[t.id];
                    cv::putText(detection_frame, idStr.str(), cv::Point(detection_x1 + 10, detection_y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
                    cv::putText(tracker_frame, idStr.str(), cv::Point(tracker_x1 + 10, tracker_y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

                    cv::rectangle(detection_frame, cv::Rect(cv::Point(detection_x1, detection_y1), cv::Point(detection_x2, detection_y2)), color, cv::FONT_HERSHEY_SIMPLEX);
                    cv::rectangle(tracker_frame, cv::Rect(cv::Point(tracker_x1, tracker_y1), cv::Point(tracker_x2, tracker_y2)), color, cv::FONT_HERSHEY_SIMPLEX);
                }
            }

            cv::rectangle(tracker_frame, cv::Rect(cv::Point(0, 0), cv::Point(150, 80)), cv::Scalar(255, 255, 255),  cv::FILLED);

            cv::putText(tracker_frame, "Flower:", cv::Point(15, 25), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 0));
            cv::putText(tracker_frame, std::to_string(counter_.at(0)), cv::Point(105, 25), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 0));

            cv::putText(tracker_frame, "Ripe:", cv::Point(15, 45), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 0));
            cv::putText(tracker_frame, std::to_string(counter_.at(1)), cv::Point(105, 45), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 0));

            cv::putText(tracker_frame, "NotRipe:", cv::Point(15, 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 0));
            cv::putText(tracker_frame, std::to_string(counter_.at(2)), cv::Point(105, 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 0));

            sensor_msgs::ImagePtr detection_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detection_frame).toImageMsg();
            sensor_msgs::ImagePtr tracker_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tracker_frame).toImageMsg();
            pub_bbox_detection_.publish(detection_msg);
            pub_bbox_tracker_.publish(tracker_msg);
        }

    public:
        BBoxPublisher(ros::NodeHandle nh, std::string node_name) : nh_{nh},
                                                                   node_name_{node_name},
                                                                   it_{nh},
                                                                   counter_{0,0,0}

        {
            ROS_INFO_STREAM("Init " + node_name_);
            sub_tracklets_.subscribe(nh_, oakd::TRACKLET_TOPIC, 1);
            sub_detection_image_.subscribe(nh_, oakd::IMAGE_DETECTION_FRAME_TOPIC, 1);
            sub_tracker_image_.subscribe(nh_, oakd::IMAGE_TRACKER_FRAME_TOPIC, 1);
            sync_.reset(new Sync(MySyncPolicy(10), sub_tracklets_, sub_detection_image_, sub_tracker_image_));
            sync_->registerCallback(boost::bind(&BBoxPublisher::cbSyncronizer, this, _1, _2, _3));
            pub_bbox_detection_ = it_.advertise(oakd::BBOX_DETECTION_FRAME_TOPIC, 1);
            pub_bbox_tracker_ = it_.advertise(oakd::BBOX_TRACKER_FRAME_TOPIC, 1);
            
        }

        ~BBoxPublisher() = default;
    };
};
