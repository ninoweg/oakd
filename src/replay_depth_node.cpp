#include <functional>
#include <iostream>
#include <string>
#include <chrono>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "utility.hpp"

int main(int argc, char **argv)
{

    std::string leftVideoPath = "/path/to/left.mp4";
    std::string rightVideoPath = "/path/to/right.mp4";

    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and outputs
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xinLeft = pipeline.create<dai::node::XLinkIn>();
    auto xinRight = pipeline.create<dai::node::XLinkIn>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
     
    xinLeft->setStreamName("inLeft");
    xinRight->setStreamName("inRight");
    xoutDepth->setStreamName("outDepth");

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(200);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(5);
    stereo->setSubpixel(false);
    stereo->setLeftRightCheck(false);
    stereo->setExtendedDisparity(false);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);

    // Linking
    xinLeft->out.link(stereo->left);
    xinRight->out.link(stereo->right);
    stereo->depth.link(xoutDepth->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto calibrationHandler = device.readCalibration();

    // Input queue will be used to send video frames to the device.
    auto qinLeft = device.getInputQueue("inLeft");
    auto qinRight = device.getInputQueue("inRight");
    // Output queue will be used to get nn data from the video frames.
    auto qoutDepth = device.getOutputQueue("outDepth", 30, false);

    cv::Mat left_frame;
    cv::VideoCapture left_cap(leftVideoPath);
    cv::Mat right_frame;
    cv::VideoCapture right_cap(rightVideoPath);

    while (left_cap.isOpened() && right_cap.isOpened())
    {
        auto timestamp = std::chrono::steady_clock::now();
        left_cap >> left_frame;
        if (!left_frame.empty())
        {
            auto img = std::make_shared<dai::ImgFrame>();
            left_frame = resizeKeepAspectRatio(left_frame, cv::Size(1280, 720), cv::Scalar(0));
            toPlanar(left_frame, img->getData());
            img->setTimestamp(timestamp);
            img->setWidth(1280);
            img->setHeight(720);
            img->setType(dai::ImgFrame::Type::YUV420p);
            img->getCvFrame();
            qinLeft->send(img);
        }

        right_cap >> right_frame;
        if (!right_frame.empty())
        {
            auto img = std::make_shared<dai::ImgFrame>();
            right_frame = resizeKeepAspectRatio(right_frame, cv::Size(1280, 720), cv::Scalar(0));
            toPlanar(right_frame, img->getData());
            img->setTimestamp(timestamp);
            img->setWidth(1280);
            img->setHeight(720);
            img->setType(dai::ImgFrame::Type::YUV420p);
            qinRight->send(img);
        }

        if(qoutDepth->has())
        {
            std::cout << "received depth\n";
        }
    }

    right_cap.release();
    left_cap.release();

    return 0;
}
