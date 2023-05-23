#include <ros/ros.h>
#include "bbox_publisher.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "bbox_publisher_node");

    ros::NodeHandle nh("~");
    oakd::BBoxPublisher bboxPub(nh, "bbox_publisher");

    ros::spin();
    
    return 0;
}