#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>
#include <string>
#include <chrono>

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <type_traits>
#include <utility>

#include "camera_info_manager/camera_info_manager.h"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/TrackletConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "ros/ros.h"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "utility.hpp"

const std::vector<std::string> labelMap = {
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle pnh("~");

    std::string tfPrefix;
    std::string rgbVideoPath, leftVideoPath, rightVideoPath, previewVideoPath, nnPath;
    bool lrcheck, extended, syncNN, subpixel;

    int badParams = 0;
    int confidence = 200;
    int LRchecktresh = 5;
    int previewHeight = 416;
    int previewWidth = 416;
    int rgbHeight = 1080;
    int rgbWidth = 1920;
    int monoHeight = 720;
    int monoWidth = 1280;
    int rgbScaleDinominator, rgbScaleNumerator;

    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("subpixel", subpixel);
    badParams += !pnh.getParam("confidence", confidence);
    badParams += !pnh.getParam("LRchecktresh", LRchecktresh);
    badParams += !pnh.getParam("lrcheck", lrcheck);
    badParams += !pnh.getParam("extended", extended);
    badParams += !pnh.getParam("nnPath", nnPath);
    badParams += !pnh.getParam("previewHeight", previewHeight);
    badParams += !pnh.getParam("previewWidth", previewWidth);
    badParams += !pnh.getParam("rgbHeight", rgbHeight);
    badParams += !pnh.getParam("rgbWidth", rgbWidth);
    badParams += !pnh.getParam("monoHeight", monoHeight);
    badParams += !pnh.getParam("monoWidth", monoWidth);
    badParams += !pnh.getParam("rgbScaleNumerator", rgbScaleNumerator);
    badParams += !pnh.getParam("rgbScaleDinominator", rgbScaleDinominator);
    badParams += !pnh.getParam("rgbVideoPath", rgbVideoPath);
    badParams += !pnh.getParam("leftVideoPath", leftVideoPath);
    badParams += !pnh.getParam("rightVideoPath", rightVideoPath);
    badParams += !pnh.getParam("previewVideoPath", previewVideoPath);

    rgbWidth = rgbWidth * rgbScaleNumerator / rgbScaleDinominator;
    rgbHeight = rgbHeight * rgbScaleNumerator / rgbScaleDinominator;

    if (badParams > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    // Print which blob we are using
    ROS_INFO_STREAM("Using blob at path: " + nnPath);
    ROS_INFO_STREAM("Using preview at path: " + previewVideoPath);
    ROS_INFO_STREAM("Using rgb at path: " + rgbVideoPath);
    ROS_INFO_STREAM("Using right at path: " + rightVideoPath);
    ROS_INFO_STREAM("Using left at path: " + leftVideoPath);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and outputs
    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto xinPreview = pipeline.create<dai::node::XLinkIn>();
    auto xinRgb = pipeline.create<dai::node::XLinkIn>();
    auto xinLeft = pipeline.create<dai::node::XLinkIn>();
    auto xinRight = pipeline.create<dai::node::XLinkIn>();

    auto xoutPreview = pipeline.create<dai::node::XLinkOut>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutDetections = pipeline.create<dai::node::XLinkOut>();
    auto xoutTracklets = pipeline.create<dai::node::XLinkOut>();

    xinPreview->setStreamName("inPreview");
    xinRgb->setStreamName("inRgb");
    xinLeft->setStreamName("inLeft");
    xinRight->setStreamName("inRight");

    xoutPreview->setStreamName("preview");
    xoutRgb->setStreamName("rgb");
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutDepth->setStreamName("depth");
    xoutDetections->setStreamName("detections");
    xoutTracklets->setStreamName("tracklets");

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setSubpixel(subpixel);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);

    // Properties
    nn->setConfidenceThreshold(0.5);
    nn->setBlobPath(nnPath);
    nn->setNumInferenceThreads(2);
    nn->input.setBlocking(false);

    // Linking
    xinPreview->out.link(nn->input);
    nn->out.link(xoutDetections->input);
    nn->passthrough.link(xoutPreview->input);
    xinRgb->out.link(xoutRgb->input);
    xinLeft->out.link(stereo->left);
    xinRight->out.link(stereo->right);
    stereo->rectifiedLeft.link(xoutLeft->input);
    stereo->rectifiedRight.link(xoutRight->input);
    stereo->depth.link(xoutDepth->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto calibrationHandler = device.readCalibration();

    // Input queue will be used to send video frames to the device.
    auto qinPreview = device.getInputQueue("inPreview");
    auto qinRgb = device.getInputQueue("inRgb");
    auto qinLeft = device.getInputQueue("inLeft");
    auto qinRight = device.getInputQueue("inRight");

    // Output queue will be used to get nn data from the video frames.
    auto qoutPreview = device.getOutputQueue("preview", 4, false);
    auto qoutRgb = device.getOutputQueue("rgb", 4, false);
    auto qoutLeft = device.getOutputQueue("left", 4, false);
    auto qoutRight = device.getOutputQueue("right", 4, false);
    auto qoutTracklets = device.getOutputQueue("tracklets", 4, false);
    auto qoutDepth = device.getOutputQueue("depth", 30, false);
    auto qoutDetections = device.getOutputQueue("detections", 4, false);

    // Preview
    dai::rosBridge::ImageConverter previewConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto previewCameraInfo = previewConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, previewWidth, previewHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> previewPublish(
        qoutPreview,
        pnh,
        std::string("color/preview/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &previewConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        previewCameraInfo,
        "color/preview");
    previewPublish.addPublisherCallback();

    // Left
    dai::rosBridge::ImageConverter monoConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto leftCameraInfo = monoConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoWidth);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(
        qoutLeft,
        pnh,
        std::string("mono/left/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &monoConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        leftCameraInfo,
        "mono/left");
    leftPublish.addPublisherCallback();

    // Right
    auto rightCameraInfo = monoConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoWidth);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(
        qoutRight,
        pnh,
        std::string("mono/right/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &monoConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rightCameraInfo,
        "mono/right");
    rightPublish.addPublisherCallback();

    // RGB
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, rgbWidth, rgbHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
        qoutRgb,
        pnh,
        std::string("color/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rgbCameraInfo,
        "color");
    rgbPublish.addPublisherCallback();

    // Depth
    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto depthCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(
        qoutDepth,
        pnh,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &depthConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        depthCameraInfo,
        "stereo");
    depthPublish.addPublisherCallback();

    // SpatialDetections
    dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", previewWidth, previewHeight, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
        qoutDetections,
        pnh,
        std::string("detections/yolov4_spatial"),
        std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        30);
    detectionPublish.addPublisherCallback();

    // Tracklets
    dai::rosBridge::TrackletConverter trackConverter(tfPrefix + "_rgb_camera_optical_frame", previewWidth, previewHeight, rgbWidth, rgbHeight, false, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackletArray, dai::Tracklets> trackletPublish(
        qoutTracklets,
        pnh,
        std::string("tracker/tracklets"),
        std::bind(&dai::rosBridge::TrackletConverter::toRosMsg, &trackConverter, std::placeholders::_1, std::placeholders::_2),
        30);
    trackletPublish.addPublisherCallback();

    cv::Mat preview_frame;
    cv::VideoCapture preview_cap(previewVideoPath);
    cv::Mat rgb_frame;
    cv::VideoCapture rgb_cap(rgbVideoPath);
    cv::Mat left_frame;
    cv::VideoCapture left_cap(leftVideoPath);
    cv::Mat right_frame;
    cv::VideoCapture right_cap(rightVideoPath);

    while (ros::ok() && preview_cap.isOpened() && rgb_cap.isOpened() && left_cap.isOpened() && right_cap.isOpened())
    {
        // Read frame from video
        preview_cap >> preview_frame;
        if (!preview_frame.empty())
        {
            auto img = std::make_shared<dai::ImgFrame>();
            preview_frame = resizeKeepAspectRatio(preview_frame, cv::Size(previewWidth, previewHeight), cv::Scalar(0));
            toPlanar(preview_frame, img->getData());
            img->setTimestamp(std::chrono::steady_clock::now());
            img->setWidth(previewWidth);
            img->setHeight(previewHeight);
            img->setType(dai::ImgFrame::Type::BGR888p);
            img->setInstanceNum(0);
            qinPreview->send(img);
        }

        rgb_cap >> rgb_frame;
        if (!rgb_frame.empty())
        {
            auto img = std::make_shared<dai::ImgFrame>();
            rgb_frame = resizeKeepAspectRatio(rgb_frame, cv::Size(rgbWidth, rgbHeight), cv::Scalar(0));
            toPlanar(rgb_frame, img->getData());
            img->setTimestamp(std::chrono::steady_clock::now());
            img->setWidth(rgbWidth);
            img->setHeight(rgbHeight);
            img->setType(dai::ImgFrame::Type::BGR888p);
            img->setInstanceNum(0);
            qinRgb->send(img);
        }

        left_cap >> left_frame;
        right_cap >> right_frame;
        if (!left_frame.empty() && !right_frame.empty())
        {
            auto timestamp = std::chrono::steady_clock::now();
            
            auto left_img = std::make_shared<dai::ImgFrame>();
            left_frame = resizeKeepAspectRatio(left_frame, cv::Size(1280, 720), cv::Scalar(0));
            toPlanar(left_frame, left_img->getData());
            left_img->setTimestamp(timestamp);
            left_img->setWidth(1280);
            left_img->setHeight(720);
            left_img->setType(dai::ImgFrame::Type::YUV420p);
            left_img->getCvFrame();
            left_img->setInstanceNum(1);
            qinLeft->send(left_img);

            auto right_img = std::make_shared<dai::ImgFrame>();
            right_frame = resizeKeepAspectRatio(right_frame, cv::Size(1280, 720), cv::Scalar(0));
            toPlanar(right_frame, right_img->getData());
            right_img->setTimestamp(timestamp);
            right_img->setWidth(1280);
            right_img->setHeight(720);
            right_img->setType(dai::ImgFrame::Type::YUV420p);
            right_img->setInstanceNum(2);
            qinRight->send(right_img);
        }

        ros::spinOnce();
    }

    right_cap.release();
    left_cap.release();
    rgb_cap.release();
    preview_cap.release();

    return 0;
}
