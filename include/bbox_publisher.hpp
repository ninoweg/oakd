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

#include "common.hpp"

namespace oakd
{
    class BBoxPublisher
    {
    private:
        ros::NodeHandle nh_;
        std::string node_name_;
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

                cv::putText(detection_frame, labelStr, cv::Point(detection_x1 + 10, detection_y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                cv::putText(tracker_frame, labelStr, cv::Point(tracker_x1 + 10, tracker_y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                std::stringstream idStr;
                idStr << "ID: " << t.id;
                cv::putText(detection_frame, idStr.str(), cv::Point(detection_x1 + 10, detection_y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                cv::putText(tracker_frame, idStr.str(), cv::Point(tracker_x1 + 10, tracker_y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                std::stringstream statusStr;
                statusStr << "Status: " << t.status;
                cv::putText(detection_frame, statusStr.str(), cv::Point(detection_x1 + 10, detection_y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                cv::putText(tracker_frame, statusStr.str(), cv::Point(tracker_x1 + 10, tracker_y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                std::stringstream depthX;
                depthX << "X: " << (float)t.spatialCoordinates.x << " m";
                cv::putText(detection_frame, depthX.str(), cv::Point(detection_x1 + 10, detection_y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                cv::putText(tracker_frame, depthX.str(), cv::Point(tracker_x1 + 10, tracker_y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                std::stringstream depthY;
                depthY << "Y: " << (float)t.spatialCoordinates.y << " m";
                cv::putText(detection_frame, depthY.str(), cv::Point(detection_x1 + 10, detection_y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                cv::putText(tracker_frame, depthY.str(), cv::Point(tracker_x1 + 10, tracker_y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                std::stringstream depthZ;
                depthZ << "Z: " << (float)t.spatialCoordinates.z << " m";
                cv::putText(detection_frame, depthZ.str(), cv::Point(detection_x1 + 10, detection_y1 + 95), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                cv::putText(tracker_frame, depthZ.str(), cv::Point(tracker_x1 + 10, tracker_y1 + 95), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                cv::rectangle(detection_frame, cv::Rect(cv::Point(detection_x1, detection_y1), cv::Point(detection_x2, detection_y2)), color, cv::FONT_HERSHEY_SIMPLEX);
                cv::rectangle(tracker_frame, cv::Rect(cv::Point(tracker_x1, tracker_y1), cv::Point(tracker_x2, tracker_y2)), color, cv::FONT_HERSHEY_SIMPLEX);
            }

            sensor_msgs::ImagePtr detection_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detection_frame).toImageMsg();
            sensor_msgs::ImagePtr tracker_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tracker_frame).toImageMsg();
            pub_bbox_detection_.publish(detection_msg);
            pub_bbox_tracker_.publish(tracker_msg);
        }

    public:
        BBoxPublisher(ros::NodeHandle nh, std::string node_name) : nh_{nh},
                                                                   node_name_{node_name},
                                                                   it_{nh}
                                                        
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
