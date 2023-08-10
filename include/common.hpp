#pragma once
#include <string>
#include <vector>

namespace oakd
{
    const std::string publisher = "replay_publisher";

    const std::string TRACKLET_TOPIC = "/" + publisher + "/tracker/tracklets";
    const std::string CAMERA_INFO_TOPIC = "/" + publisher + "/stereo/camera_info";
    const std::string IMAGE_TRACKER_FRAME_TOPIC = "/" + publisher + "/color/image";
    const std::string IMAGE_DETECTION_FRAME_TOPIC = "/" + publisher + "/color/preview/image";

    const std::string MARKER_TOPIC = "/" + publisher + "/marker_publisher/markers";
    const std::string BBOX_TRACKER_FRAME_TOPIC = "/" + publisher + "/bbox_publisher/tracker_frame/image";
    const std::string BBOX_DETECTION_FRAME_TOPIC = "/" + publisher + "/bbox_publisher/detection_frame/image";
}

// static const std::vector<std::string> labelMap = {
//     "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
//     "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
//     "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
//     "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
//     "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
//     "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
//     "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
//     "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
//     "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
// };

static const std::vector<std::string> labelMap = {
    "Flower",
    "Strawberry",
    "StrawberryNotReady"
};