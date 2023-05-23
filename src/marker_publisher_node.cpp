#include <ros/ros.h>
#include "marker_publisher.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_publisher_node");

    ros::NodeHandle nh("~");
    oakd::MarkerPublisher mp(nh, "marker_publisher");

    ros::spin();
    
    return 0;
}