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
#include <vector>

#include "camera_info_manager/camera_info_manager.h"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/TrackletConverter.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "utility.hpp"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};

const std::vector<std::string> label_map = {"Flower", "Strawberry", "StrawberryNotReady"};

std::tuple<dai::Pipeline, int, int, int, int> createPipeline(bool enableDepth,
                                                             bool enableSpatialDetection,
                                                             bool lrcheck,
                                                             bool extended,
                                                             bool subpixel,
                                                             bool rectify,
                                                             bool depth_aligned,
                                                             int stereo_fps,
                                                             int confidence,
                                                             int LRchecktresh,
                                                             int detectionClassesCount,
                                                             std::string stereoResolution,
                                                             std::string rgbResolutionStr,
                                                             int rgbScaleNumerator,
                                                             int rgbScaleDinominator,
                                                             int previewWidth,
                                                             int previewHeight,
                                                             bool syncNN,
                                                             std::string nnPath)
{
    // Create pipeline
    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);

    // Define source and outputs
    auto monoLeft = pipeline.create<dai::node::XLinkIn>();
    auto monoRight = pipeline.create<dai::node::XLinkIn>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto imgManip = pipeline.create<dai::node::ImageManip>();

    imgManip->setMaxOutputFrameSize(3000000);
    imgManip->initialConfig.setResize(previewWidth, previewHeight);
    imgManip->setKeepAspectRatio(true);

    xoutDepth->setStreamName("depth");

    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    int stereoWidth, stereoHeight, rgbWidth, rgbHeight;
    if (stereoResolution == "720p")
    {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        stereoWidth = 1280;
        stereoHeight = 720;
    }
    else if (stereoResolution == "400p")
    {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        stereoWidth = 640;
        stereoHeight = 400;
    }
    else if (stereoResolution == "800p")
    {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        stereoWidth = 1280;
        stereoHeight = 800;
    }
    else if (stereoResolution == "480p")
    {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        stereoWidth = 640;
        stereoHeight = 480;
    }
    else
    {
        ROS_ERROR("Invalid parameter. -> monoResolution: %s", stereoResolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeft->setStreamName("inLeft");
    monoRight->setStreamName("inRight");

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    auto camRgb = pipeline.create<dai::node::XLinkIn>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");

    dai::node::ColorCamera::Properties::SensorResolution rgbResolution;

    if (rgbResolutionStr == "1080p")
    {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P;
        rgbWidth = 1920;
        rgbHeight = 1080;
    }
    else if (rgbResolutionStr == "4K")
    {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_4_K;
        rgbWidth = 3840;
        rgbHeight = 2160;
    }
    else if (rgbResolutionStr == "12MP")
    {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_12_MP;
        rgbWidth = 4056;
        rgbHeight = 3040;
    }
    else if (rgbResolutionStr == "13MP")
    {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_13_MP;
        rgbWidth = 4208;
        rgbHeight = 3120;
    }
    else
    {
        ROS_ERROR("Invalid parameter. -> rgbResolution: %s", rgbResolutionStr.c_str());
        throw std::runtime_error("Invalid color camera resolution.");
    }

    rgbWidth = rgbWidth * rgbScaleNumerator / rgbScaleDinominator;
    rgbHeight = rgbHeight * rgbScaleNumerator / rgbScaleDinominator;

    stereo->setOutputSize(rgbWidth, rgbHeight);

    // std::cout << (rgbWidth % 2 == 0 && rgbHeight % 3 == 0) << std::endl;
    // assert(("Needs Width to be multiple of 2 and height to be multiple of 3 since the Image is NV12 format here.", (rgbWidth % 2 == 0 && rgbHeight % 3 ==
    // 0)));

    if (rgbWidth % 16 != 0)
    {
        if (rgbResolution == dai::node::ColorCamera::Properties::SensorResolution::THE_12_MP)
        {
            ROS_ERROR_STREAM("RGB Camera width should be multiple of 16. Please choose a different scaling factor."
                             << std::endl
                             << "Here are the scalng options that works for 12MP with depth aligned" << std::endl
                             << "4056 x 3040 *  2/13 -->  624 x  468" << std::endl
                             << "4056 x 3040 *  2/39 -->  208 x  156" << std::endl
                             << "4056 x 3040 *  2/51 -->  160 x  120" << std::endl
                             << "4056 x 3040 *  4/13 --> 1248 x  936" << std::endl
                             << "4056 x 3040 *  4/26 -->  624 x  468" << std::endl
                             << "4056 x 3040 *  4/29 -->  560 x  420" << std::endl
                             << "4056 x 3040 *  4/35 -->  464 x  348" << std::endl
                             << "4056 x 3040 *  4/39 -->  416 x  312" << std::endl
                             << "4056 x 3040 *  6/13 --> 1872 x 1404" << std::endl
                             << "4056 x 3040 *  6/39 -->  624 x  468" << std::endl
                             << "4056 x 3040 *  7/25 --> 1136 x  852" << std::endl
                             << "4056 x 3040 *  8/26 --> 1248 x  936" << std::endl
                             << "4056 x 3040 *  8/39 -->  832 x  624" << std::endl
                             << "4056 x 3040 *  8/52 -->  624 x  468" << std::endl
                             << "4056 x 3040 *  8/58 -->  560 x  420" << std::endl
                             << "4056 x 3040 * 10/39 --> 1040 x  780" << std::endl
                             << "4056 x 3040 * 10/59 -->  688 x  516" << std::endl
                             << "4056 x 3040 * 12/17 --> 2864 x 2146" << std::endl
                             << "4056 x 3040 * 12/26 --> 1872 x 1404" << std::endl
                             << "4056 x 3040 * 12/39 --> 1248 x  936" << std::endl
                             << "4056 x 3040 * 13/16 --> 3296 x 2470" << std::endl
                             << "4056 x 3040 * 14/39 --> 1456 x 1092" << std::endl
                             << "4056 x 3040 * 14/50 --> 1136 x  852" << std::endl
                             << "4056 x 3040 * 14/53 --> 1072 x  804" << std::endl
                             << "4056 x 3040 * 16/39 --> 1664 x 1248" << std::endl
                             << "4056 x 3040 * 16/52 --> 1248 x  936" << std::endl);
        }
        else
        {
            ROS_ERROR_STREAM("RGB Camera width should be multiple of 16. Please choose a different scaling factor.");
        }
        throw std::runtime_error("Adjust RGB Camaera scaling.");
    }

    if (rgbWidth > stereoWidth || rgbHeight > stereoHeight)
    {
        ROS_WARN_STREAM(
            "RGB Camera resolution is heigher than the configured stereo resolution. Upscaling the stereo depth/disparity to match RGB camera resolution.");
    }
    else if (rgbWidth > stereoWidth || rgbHeight > stereoHeight)
    {
        ROS_WARN_STREAM(
            "RGB Camera resolution is heigher than the configured stereo resolution. Downscaling the stereo depth/disparity to match RGB camera "
            "resolution.");
    }

    if (previewWidth > rgbWidth or previewHeight > rgbHeight)
    {
        ROS_ERROR_STREAM(
            "Preview Image size should be smaller than the scaled resolution. Please adjust the scale parameters or the preview size accordingly.");
        throw std::runtime_error("Invalid Image Size");
    }

    camRgb->setStreamName("inRgb");

    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
    auto xoutNN = pipeline.create<dai::node::XLinkOut>();
    auto xoutPreview = pipeline.create<dai::node::XLinkOut>();
    xoutPreview->setStreamName("preview");
    xoutNN->setStreamName("detections");

    // SpatialDetections
    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.75f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(10000);
    // spatialDetectionNetwork->setSpatialCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::MIN);

    // yolo specific parameters
    spatialDetectionNetwork->setNumClasses(3);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10.0, 13.0, 16.0, 30.0, 33.0, 23.0, 30.0, 61.0, 62.0, 45.0, 59.0, 119.0, 116.0, 90.0, 156.0, 198.0, 373.0, 326.0});
    // spatialDetectionNetwork->setAnchorMasks({{"side52", {0, 1, 2}}, {"side26", {3, 4, 5}}, {"side13", {6, 7, 8}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);
    spatialDetectionNetwork->setAnchorMasks({{"side44", {0, 1, 2}}, {"side22", {3, 4, 5}}, {"side11", {6, 7, 8}}});

    // Link plugins CAM -> NN -> XLINK
    camRgb->out.link(imgManip->inputImage);
    imgManip->out.link(spatialDetectionNetwork->input);

    if (syncNN)
        spatialDetectionNetwork->passthrough.link(xoutPreview->input);
    else
        camRgb->out.link(xoutPreview->input);

    stereo->depth.link(spatialDetectionNetwork->inputDepth);

    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();
    auto xoutTracker = pipeline.create<dai::node::XLinkOut>();
    xoutTracker->setStreamName("tracklets");

    // ObjectTracker
    objectTracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
    objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::UNIQUE_ID);
    objectTracker->inputTrackerFrame.setBlocking(false);
    objectTracker->setTrackingPerClass(true);
    // objectTracker->setTrackerThreshold(0.8);

    // Link ColorCamera
    camRgb->out.link(objectTracker->inputTrackerFrame);

    // Link Mono
    stereo->setRectifyEdgeFillColor(0);
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    // Link SpatialDetectionNetwork
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
    spatialDetectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    spatialDetectionNetwork->out.link(objectTracker->inputDetections);

    // Link ObjectTracker
    objectTracker->passthroughDetectionFrame.link(xoutPreview->input);
    objectTracker->passthroughDetections.link(xoutNN->input);
    objectTracker->passthroughTrackerFrame.link(xoutRgb->input);
    objectTracker->out.link(xoutTracker->input);
    objectTracker->inputTrackerFrame.setBlocking(false);
    objectTracker->inputTrackerFrame.setQueueSize(2);

    std::cout << stereoWidth << " " << stereoHeight << " " << rgbWidth << " " << rgbHeight << std::endl;
    return std::make_tuple(pipeline, rgbWidth, rgbHeight, stereoWidth, stereoHeight);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "replay_node");
    ros::NodeHandle pnh("~");

    std::string tfPrefix, mode, mxId, resourceBaseFolder, nnPath, rgbVideoPath, leftVideoPath, rightVideoPath, bagPath;
    std::string monoResolution = "720p", rgbResolution = "1080p";
    int badParams = 0, stereo_fps, confidence, LRchecktresh, imuModeParam, detectionClassesCount, expTime, sensIso;
    int rgbScaleNumerator, rgbScaleDinominator, previewWidth, previewHeight;
    bool lrcheck, extended, subpixel, enableDepth, rectify, depth_aligned, manualExposure;
    bool enableSpatialDetection, enableDotProjector, enableFloodLight;
    bool usb2Mode, poeMode, syncNN;
    double angularVelCovariance, linearAccelCovariance;
    double dotProjectormA, floodLightmA;
    std::string nnName; // Set your blob name for the model here

    badParams += !pnh.getParam("mxId", mxId);
    badParams += !pnh.getParam("usb2Mode", usb2Mode);
    badParams += !pnh.getParam("poeMode", poeMode);
    badParams += !pnh.getParam("resourceBaseFolder", resourceBaseFolder);

    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("mode", mode);
    badParams += !pnh.getParam("imuMode", imuModeParam);

    badParams += !pnh.getParam("lrcheck", lrcheck);
    badParams += !pnh.getParam("extended", extended);
    badParams += !pnh.getParam("subpixel", subpixel);
    badParams += !pnh.getParam("rectify", rectify);

    badParams += !pnh.getParam("depth_aligned", depth_aligned);
    badParams += !pnh.getParam("stereo_fps", stereo_fps);
    badParams += !pnh.getParam("confidence", confidence);
    badParams += !pnh.getParam("LRchecktresh", LRchecktresh);
    badParams += !pnh.getParam("monoResolution", monoResolution);
    badParams += !pnh.getParam("rgbResolution", rgbResolution);
    badParams += !pnh.getParam("manualExposure", manualExposure);
    badParams += !pnh.getParam("expTime", expTime);
    badParams += !pnh.getParam("sensIso", sensIso);

    badParams += !pnh.getParam("rgbScaleNumerator", rgbScaleNumerator);
    badParams += !pnh.getParam("rgbScaleDinominator", rgbScaleDinominator);
    badParams += !pnh.getParam("previewWidth", previewWidth);
    badParams += !pnh.getParam("previewHeight", previewHeight);

    badParams += !pnh.getParam("angularVelCovariance", angularVelCovariance);
    badParams += !pnh.getParam("linearAccelCovariance", linearAccelCovariance);
    badParams += !pnh.getParam("enableSpatialDetection", enableSpatialDetection);
    badParams += !pnh.getParam("detectionClassesCount", detectionClassesCount);
    badParams += !pnh.getParam("syncNN", syncNN);

    // Applies only to PRO model
    badParams += !pnh.getParam("enableDotProjector", enableDotProjector);
    badParams += !pnh.getParam("enableFloodLight", enableFloodLight);
    badParams += !pnh.getParam("dotProjectormA", dotProjectormA);
    badParams += !pnh.getParam("floodLightmA", floodLightmA);

    badParams += !pnh.getParam("rgbVideoPath", rgbVideoPath);
    badParams += !pnh.getParam("leftVideoPath", leftVideoPath);
    badParams += !pnh.getParam("rightVideoPath", rightVideoPath);
    badParams += !pnh.getParam("bagPath", bagPath);

    // Print which blob we are using
    ROS_INFO_STREAM("Using blob at path: " + nnPath);
    ROS_INFO_STREAM("Using rgb at path: " + rgbVideoPath);
    ROS_INFO_STREAM("Using right at path: " + rightVideoPath);
    ROS_INFO_STREAM("Using left at path: " + leftVideoPath);

    if (badParams > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }
    std::string nnParam;
    pnh.getParam("nnName", nnParam);
    if (nnParam != "x")
    {
        pnh.getParam("nnName", nnName);
    }

    if (resourceBaseFolder.empty())
    {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
    }
    nnPath = resourceBaseFolder + "/" + nnName;
    std::cout << " nnPath ,, " << nnPath << std::endl;
    if (mode == "depth")
    {
        enableDepth = true;
    }
    else
    {
        enableDepth = false;
    }

    dai::Pipeline pipeline;
    int rgbWidth, rgbHeight, stereoWidth, stereoHeight, width, height;
    bool isDeviceFound = false;
    std::tie(pipeline, rgbWidth, rgbHeight, stereoWidth, stereoHeight) = createPipeline(enableDepth,
                                                                                        enableSpatialDetection,
                                                                                        lrcheck,
                                                                                        extended,
                                                                                        subpixel,
                                                                                        rectify,
                                                                                        depth_aligned,
                                                                                        stereo_fps,
                                                                                        confidence,
                                                                                        LRchecktresh,
                                                                                        detectionClassesCount,
                                                                                        monoResolution,
                                                                                        rgbResolution,
                                                                                        rgbScaleNumerator,
                                                                                        rgbScaleDinominator,
                                                                                        previewWidth,
                                                                                        previewHeight,
                                                                                        syncNN,
                                                                                        nnPath);

    width = rgbWidth;
    height = rgbHeight;

    // Device
    std::shared_ptr<dai::Device> device;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    std::cout << "Listing available devices..." << std::endl;
    for (auto deviceInfo : availableDevices)
    {
        std::cout << "Device Mx ID: " << deviceInfo.getMxId() << std::endl;
        if (deviceInfo.getMxId() == mxId)
        {
            if (deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER)
            {
                isDeviceFound = true;
                if (poeMode)
                {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo);
                }
                else
                {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo, usb2Mode);
                }
                break;
            }
            else if (deviceInfo.state == X_LINK_BOOTED)
            {
                throw std::runtime_error("ros::NodeHandle() from Node \"" + pnh.getNamespace() + "\" DepthAI Device with MxId  \"" + mxId + "\" is already booted on different process.  \"");
            }
        }
        else if (mxId.empty())
        {
            isDeviceFound = true;
            device = std::make_shared<dai::Device>(pipeline);
        }
    }

    if (!isDeviceFound)
    {
        throw std::runtime_error("ros::NodeHandle() from Node \"" + pnh.getNamespace() + "\" DepthAI Device with MxId  \"" + mxId + "\" not found.  \"");
    }

    if (!poeMode)
    {
        std::cout << "Device USB status: " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << std::endl;
    }

    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if (enableDepth)
    {
        stereoQueue = device->getOutputQueue("depth", 30, false);
    }
    else
    {
        stereoQueue = device->getOutputQueue("disparity", 30, false);
    }

    auto calibrationHandler = device->readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if (height > 480 && boardName == "OAK-D-LITE" && depth_aligned == false)
    {
        width = 640;
        height = 480;
    }

    // Input queue will be used to send video frames to the device.
    auto qinRgb = device->getInputQueue("inRgb");
    auto qinLeft = device->getInputQueue("inLeft");
    auto qinRight = device->getInputQueue("inRight");

    qinRgb->setBlocking(false);
    qinLeft->setBlocking(false);
    qinRight->setBlocking(false);

    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    const std::string leftPubName = rectify ? std::string("left/image_rect") : std::string("left/image_raw");
    const std::string rightPubName = rectify ? std::string("right/image_rect") : std::string("right/image_raw");

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);

    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, width, height);

    auto depthCameraInfo =
        depth_aligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, width, height) : rightCameraInfo;

    auto depthconverter = depth_aligned ? rgbConverter : rightconverter;
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(stereoQueue,
                                                                                    pnh,
                                                                                    std::string("stereo/depth"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                              &depthconverter, // since the converter has the same frame name
                                                                                                               // and image type is also same we can reuse it
                                                                                              std::placeholders::_1,
                                                                                              std::placeholders::_2),
                                                                                    30,
                                                                                    depthCameraInfo,
                                                                                    "stereo");
    depthPublish.addPublisherCallback();

    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, width, height);
    auto imgQueue = device->getOutputQueue("rgb", 30, false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
        imgQueue,
        pnh,
        std::string("color/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rgbCameraInfo,
        "color");
    rgbPublish.addPublisherCallback();

    auto previewQueue = device->getOutputQueue("preview", 30, false);
    auto detectionQueue = device->getOutputQueue("detections", 30, false);
    auto trackletQueue = device->getOutputQueue("tracklets", 30, false);
    auto previewCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, previewWidth, previewHeight);

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> previewPublish(
        previewQueue,
        pnh,
        std::string("color/preview/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        previewCameraInfo,
        "color/preview");
    previewPublish.addPublisherCallback();

    dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", previewWidth, previewHeight, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
        detectionQueue,
        pnh,
        std::string("color/yolov4_Spatial_detections"),
        std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        30);
    detectionPublish.addPublisherCallback();

    dai::rosBridge::TrackletConverter trackConverter(tfPrefix + "_rgb_camera_optical_frame", previewWidth, previewHeight, width, height, false, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackletArray, dai::Tracklets> trackletPublish(
        trackletQueue,
        pnh,
        std::string("tracker/tracklets"),
        std::bind(&dai::rosBridge::TrackletConverter::toRosMsg, &trackConverter, std::placeholders::_1, std::placeholders::_2),
        30);
    trackletPublish.addPublisherCallback();

    cv::Mat rgb_frame;
    cv::VideoCapture rgb_cap(rgbVideoPath);
    cv::Mat left_frame;
    cv::VideoCapture left_cap(leftVideoPath);
    cv::Mat right_frame;
    cv::VideoCapture right_cap(rightVideoPath);

    rosbag::Bag bag;
    bag.open(bagPath, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/odom"));
    topics.push_back(std::string("/tf"));
    topics.push_back(std::string("/imu"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::vector<nav_msgs::Odometry::ConstPtr> odom_msgs;
    std::vector<tf2_msgs::TFMessage::ConstPtr> tf_msgs;
    std::vector<sensor_msgs::Imu::ConstPtr> imu_msgs;

    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        nav_msgs::Odometry::ConstPtr j = m.instantiate<nav_msgs::Odometry>();
        if (j != nullptr)
            odom_msgs.push_back(j);

        tf2_msgs::TFMessage::ConstPtr k = m.instantiate<tf2_msgs::TFMessage>();
        if (k != nullptr)
            tf_msgs.push_back(k);

        sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
        if (i != nullptr)
            imu_msgs.push_back(i);
    }

    auto o{0}, i{0}, t{0}, m{0};
    ros::Publisher pub_odom = pnh.advertise<nav_msgs::Odometry>("/odom", 1);
    ros::Publisher pub_imu = pnh.advertise<sensor_msgs::Imu>("/imu", 1);
    tf::TransformBroadcaster br;

    ros::Time last_frame{ros::Time::now()}, start{ros::Time::now()};
    uint64_t duration{0};

    while (ros::ok() && rgb_cap.isOpened() && left_cap.isOpened() && right_cap.isOpened())
    {
        if (ros::Time::now().toNSec() - last_frame.toNSec() > (1 / static_cast<double>(stereo_fps)) * 1e9 - duration)
        {
            start = ros::Time::now();

            auto timestamp = std::chrono::steady_clock::now();

            if (!odom_msgs.empty() && o <= odom_msgs.size() - 2)
            {
                auto msg = *odom_msgs.at(o);
                msg.header.stamp = ros::Time::now();
                msg.child_frame_id = "base_link";
                msg.header.frame_id = "odom";
                pub_odom.publish(msg);
                o++;
            }

            if (!imu_msgs.empty() && static_cast<int>(2.5 * m) <= imu_msgs.size() - 2)
            {
                auto msg = *imu_msgs.at(static_cast<int>(2.5 * m));
                msg.header.stamp = ros::Time::now();
                pub_imu.publish(msg);
                m++;
            }

            if (!tf_msgs.empty() && t <= tf_msgs.size() - 2)
            {
                for (auto msg : tf_msgs.at(t)->transforms)
                {
                    tf::StampedTransform stamped_tf;
                    msg.child_frame_id = "base_link";
                    msg.header.frame_id = "odom";
                    msg.header.stamp = ros::Time::now();
                    tf::transformStampedMsgToTF(msg, stamped_tf);
                    br.sendTransform(stamped_tf);
                }
                t++;
            }

            rgb_cap >> rgb_frame;
            left_cap >> left_frame;
            right_cap >> right_frame;

            if (!rgb_frame.empty() && !left_frame.empty() && !right_frame.empty())
            {
                auto rgb_img = std::make_shared<dai::ImgFrame>();
                rgb_frame = resizeKeepAspectRatio(rgb_frame, cv::Size(width, height), cv::Scalar(0));
                toPlanar(rgb_frame, rgb_img->getData());
                rgb_img->setTimestamp(timestamp);
                rgb_img->setWidth(width);
                rgb_img->setHeight(height);
                rgb_img->setType(dai::ImgFrame::Type::BGR888p);
                rgb_img->setInstanceNum(0);
                qinRgb->send(rgb_img);

                auto left_img = std::make_shared<dai::ImgFrame>();
                left_frame = resizeKeepAspectRatio(left_frame, cv::Size(stereoWidth, stereoHeight), cv::Scalar(0));
                toPlanar(left_frame, left_img->getData());
                left_img->setTimestamp(timestamp);
                left_img->setWidth(stereoWidth);
                left_img->setHeight(stereoHeight);
                left_img->setType(dai::ImgFrame::Type::BGR888p);
                left_img->setInstanceNum(1);
                qinLeft->send(left_img);

                auto right_img = std::make_shared<dai::ImgFrame>();
                right_frame = resizeKeepAspectRatio(right_frame, cv::Size(stereoWidth, stereoHeight), cv::Scalar(0));
                toPlanar(right_frame, right_img->getData());
                right_img->setTimestamp(timestamp);
                right_img->setWidth(stereoWidth);
                right_img->setHeight(stereoHeight);
                right_img->setType(dai::ImgFrame::Type::BGR888p);
                right_img->setInstanceNum(2);
                qinRight->send(right_img);
                i++;
            }

            last_frame = ros::Time::now();
            duration = last_frame.toNSec() - start.toNSec();
        }

        ros::spinOnce();
    }

    right_cap.release();
    left_cap.release();
    rgb_cap.release();
    bag.close();

    return 0;
}
