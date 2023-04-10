#include <functional>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <depthai_ros_msgs/SpatialDetectionArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <oakd_msgs/TrackletArray.h>
#include <oakd_msgs/Tracklet.h>

// Inludes common necessary includes for development using depthai library
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/DisparityConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImuConverter.hpp>
#include <depthai_bridge/SpatialDetectionConverter.hpp>

#include "depthai/depthai.hpp"

static const std::vector<std::string> labelMap = {
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

dai::Pipeline createPipeline(int stereo_fps,
                             int rgb_fps,
                             int rgbScaleNumerator,
                             int rgbScaleDinominator,
                             int previewWidth,
                             int previewHeight,
                             std::string nnPath,
                             int rgbWidth,
                             int rgbHeight,
                             int stereoWidth,
                             int stereoHeight)
{
    // Pipeline
    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);
    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();
    auto xoutNN = pipeline.create<dai::node::XLinkOut>();
    auto xoutPreview = pipeline.create<dai::node::XLinkOut>();
    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();
    auto xoutTracker = pipeline.create<dai::node::XLinkOut>();

    controlIn->setStreamName("control");
    xoutPreview->setStreamName("preview");
    xoutRgb->setStreamName("rgb");
    xoutDepth->setStreamName("depth");
    xoutNN->setStreamName("detections");
    xoutTracker->setStreamName("tracklets");
    xoutImu->setStreamName("imu");

    // MonoCameras
    dai::node::MonoCamera::Properties::SensorResolution monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setFps(stereo_fps);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setFps(stereo_fps);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(200);   // Known to be best
    stereo->setRectifyEdgeFillColor(0);                  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(5); // Known to be best
    stereo->setLeftRightCheck(true);
    stereo->setExtendedDisparity(false);
    stereo->setSubpixel(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    // ObjectTracker
    objectTracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
    objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);

    // IMU
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20); // Get one message only for now.

    // ColorCamera
    rgbWidth = rgbWidth * rgbScaleNumerator / rgbScaleDinominator;
    rgbHeight = rgbHeight * rgbScaleNumerator / rgbScaleDinominator;

    dai::node::ColorCamera::Properties::SensorResolution rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P;
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setFps(stereo_fps);
    camRgb->setResolution(rgbResolution);
    camRgb->setIspScale(rgbScaleNumerator, rgbScaleDinominator);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setInterleaved(false);
    camRgb->setPreviewSize(previewWidth, previewHeight);
    camRgb->setFps(rgb_fps);

    // SpatialDetectionNetwork
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(10000);
    // YOLO specific parameters
    spatialDetectionNetwork->setNumClasses(80);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);

    // Link Control
    controlIn->out.link(monoRight->inputControl);
    controlIn->out.link(monoLeft->inputControl);

    // Link MonoCameras
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    // Link ColorCamera
    camRgb->isp.link(xoutRgb->input);
    camRgb->preview.link(spatialDetectionNetwork->input);

    // Link StereoCamera
    stereo->depth.link(spatialDetectionNetwork->inputDepth);

    // Link SpatialDetectionNetwork
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
    spatialDetectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    spatialDetectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);
    spatialDetectionNetwork->out.link(objectTracker->inputDetections);

    // Link ObjectTracker
    // objectTracker->setDetectionLabelsToTrack({1});
    objectTracker->passthroughDetectionFrame.link(xoutPreview->input);
    objectTracker->passthroughDetections.link(xoutNN->input);
    objectTracker->out.link(xoutTracker->input);

    // Link IMU
    imu->out.link(xoutImu->input);

    return pipeline;
}

// void addDetectionsToFrame(cv::Mat &frame, dai::Tracklets &track)
// {
//     // https://docs.luxonis.com/projects/api/en/latest/samples/ObjectTracker/spatial_object_tracker/#spatial-object-tracker-on-rgb
//     auto color = cv::Scalar(255, 255, 255);
//     auto trackletsData = track->tracklets;
//     for (auto &t : trackletsData)
//     {

//     }
// }

void timerCallback(const ros::TimerEvent &e, const std::shared_ptr<dai::DataOutputQueue> tracklets, const std::shared_ptr<dai::DataOutputQueue> preview, const ros::Publisher &pub_tracklets, const image_transport::Publisher &pub_detection_img)
{

    static size_t seq{0};
    auto imgFrame = preview->get<dai::ImgFrame>();
    cv::Mat frame = imgFrame->getCvFrame();
    auto track = tracklets->get<dai::Tracklets>();
    auto trackletsData = track->tracklets;
    oakd_msgs::TrackletArray tracklets_ros;
    auto color = cv::Scalar(255, 255, 255);

    for (auto &t : trackletsData)
    {
        auto roi = t.roi.denormalize(frame.cols, frame.rows);
        int x1 = roi.topLeft().x;
        int y1 = roi.topLeft().y;
        int x2 = roi.bottomRight().x;
        int y2 = roi.bottomRight().y;

        uint32_t labelIndex = t.label;
        std::string labelStr = std::to_string(labelIndex);
        if (labelIndex < labelMap.size())
        {
            labelStr = labelMap[labelIndex];
        }
        cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        std::stringstream idStr;
        idStr << "ID: " << t.id;
        cv::putText(frame, idStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        std::stringstream statusStr;
        statusStr << "Status: " << t.status;
        cv::putText(frame, statusStr.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        std::stringstream depthX;
        depthX << "X: " << (int)t.spatialCoordinates.x << " mm";
        cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        std::stringstream depthY;
        depthY << "Y: " << (int)t.spatialCoordinates.y << " mm";
        cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        std::stringstream depthZ;
        depthZ << "Z: " << (int)t.spatialCoordinates.z << " mm";
        cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 95), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);

        oakd_msgs::Tracklet t_ros;
        t_ros.roi.center.x = x1 + roi.width / 2;
        t_ros.roi.center.x = y1 + roi.height / 2;
        t_ros.roi.size_x = roi.width; 
        t_ros.roi.size_y = roi.height;
        t_ros.id = t.id;
        t_ros.label = t.label;
        t_ros.age = t.age;
        t_ros.status = static_cast<std::underlying_type<dai::Tracklet::TrackingStatus>::type>(t.status);
        t_ros.srcImgDetection.label = t.srcImgDetection.label;
        t_ros.srcImgDetection.confidence = t.srcImgDetection.confidence;
        t_ros.srcImgDetection.xmin = t.srcImgDetection.xmin;
        t_ros.srcImgDetection.ymin = t.srcImgDetection.ymin;
        t_ros.srcImgDetection.xmax = t.srcImgDetection.xmax;
        t_ros.srcImgDetection.ymax = t.srcImgDetection.ymax;
        t_ros.spatialCoordinates.x = t.spatialCoordinates.x;
        t_ros.spatialCoordinates.y = t.spatialCoordinates.y;
        t_ros.spatialCoordinates.z = t.spatialCoordinates.z;
        tracklets_ros.tracklets.push_back(t_ros);
    }

    tracklets_ros.header.seq = ++seq;
    tracklets_ros.header.stamp = ros::Time::now();
    tracklets_ros.header.frame_id = "oak_rgb_camera_optical_frame";
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    pub_tracklets.publish(tracklets_ros);
    pub_detection_img.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oakd");
    ros::NodeHandle pnh("~");

    image_transport::ImageTransport it(pnh);
    image_transport::Publisher pub_detection_img = it.advertise("camera/detections/image", 1);
    ros::Publisher pub_tracklets = pnh.advertise<oakd_msgs::TrackletArray>("camera/tracker/tracklets", 1);

    std::string tfPrefix = "oak";
    std::string nnPath = "/home/ubuntu/catkin_ws/src/oakd/resources/yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr.blob";
    int imuModeParam = 1;
    int previewWidth = 416;
    int previewHeight = 416;
    int rgbScaleNumerator = 2;
    int rgbScaleDinominator = 3;
    // THE_1080_P
    int rgbWidth = 1920;
    int rgbHeight = 1080;
    int rgb_fps = 15;
    // THE_400_P
    int stereoWidth = 640;
    int stereoHeight = 400;
    int stereo_fps = 15;

    double angularVelCovariance = 0.0;
    double linearAccelCovariance = 0.0;

    dai::Pipeline pipeline = createPipeline(stereo_fps,
                                            rgb_fps,
                                            rgbScaleNumerator,
                                            rgbScaleDinominator,
                                            previewWidth,
                                            previewHeight,
                                            nnPath,
                                            rgbWidth,
                                            rgbHeight,
                                            stereoWidth,
                                            stereoHeight);

    // Device
    std::shared_ptr<dai::Device> device;
    device = std::make_shared<dai::Device>(pipeline);

    // Get Input/Output Queues
    auto previewQueue = device->getOutputQueue("preview", 30, false);
    auto imgQueue = device->getOutputQueue("rgb", 30, false);
    auto stereoQueue = device->getOutputQueue("depth", 30, false);
    auto detectionQueue = device->getOutputQueue("detections", 30, false);
    auto imuQueue = device->getOutputQueue("imu", 30, false);
    auto controlQueue = device->getInputQueue("control");
    auto trackletQueue = device->getOutputQueue("tracklets", 30, false);

    auto calibrationHandler = device->readCalibration();

    // IMU
    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);
    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(
        imuQueue,
        pnh,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");
    imuPublish.addPublisherCallback();

    // RGB
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);

    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, rgbWidth, rgbHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
        imgQueue,
        pnh,
        std::string("color/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rgbCameraInfo,
        "color");
    rgbPublish.addPublisherCallback();

    // Preview
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

    // depth
    auto depthconverter = rgbConverter;
    rgbWidth = rgbWidth * rgbScaleNumerator / rgbScaleDinominator;
    rgbHeight = rgbHeight * rgbScaleNumerator / rgbScaleDinominator;
    auto depthCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, rgbWidth, rgbHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(
        stereoQueue,
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

    // SpatialDetections
    dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 416, 416, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
        detectionQueue,
        pnh,
        std::string("color/spatial_detections"),
        std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        30);
    detectionPublish.addPublisherCallback();

    // ObjectTracker
    ros::Timer timer = pnh.createTimer(ros::Duration(1.0 / rgb_fps), std::bind(&timerCallback, std::placeholders::_1, trackletQueue, previewQueue, pub_tracklets, pub_detection_img));

    ros::spin();

    return 0;
}
