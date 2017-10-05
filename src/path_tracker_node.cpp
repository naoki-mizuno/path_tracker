#include "path_tracker/path_tracker.h"

#include <ros/ros.h>

#include <thread>

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_tracker");
    ros::NodeHandle nh;

    PathTracker path_tracker;

    path_tracker.start_control_loop();

    ros::spin();

    return 0;
}
