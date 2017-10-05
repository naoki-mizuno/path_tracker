#ifndef PATH_TRACKER_PATH_TRACKER_H
#define PATH_TRACKER_PATH_TRACKER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <vector>
#include <utility>

class PathTracker {
public:
    using Pose = geometry_msgs::Pose;

    PathTracker();

    PathTracker(const PathTracker& other);

    PathTracker(PathTracker&& other) noexcept;

    ~PathTracker();

    /**
     * Returns the commanded velocity and angular velocity
     * @param pr current robot pose
     * @param p0 point on path (local start)
     * @param p1 point on path (local goal)
     * @return commanded velocity, angular velocity, point on path closest
     * to the robot
     */
    std::tuple<double, double, geometry_msgs::Point>
    get_v_w(const Pose& pr, const Pose& p0, const Pose& p1);

    void
    start_control_loop();

private:
    std::vector<Pose> path;
    nav_msgs::Path path_msg;
    ros::Subscriber path_sub;
    ros::Publisher cmd_vel_pub;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    std::string map_frame;
    std::string base_frame;

    double K_d_t, K_w, K_d_d;

    /**
     * How close (in meters) is close enough?
     */
    double close_enough;

    /**
     * How often to send cmd_vel
     */
    double update_rate;

    /* Velocity v (m/s) and Angular velocity omega (rad/s) */
    double v, w;
    /* Delta distance */
    double d_d;
    /* Delta theta */
    double d_t;

    void
    path_callback(const nav_msgs::Path& msg);

    /**
     * @return the current pose of the robot
     */
    Pose
    get_robot_pose();

    /**
     * Returns the distance between point a and b
     * @param a
     * @param b
     * @return the distance between a and b
     */
    template <typename T>
    double
    get_dist(const T& a, const T& b);
};

#endif //PATH_TRACKER_PATH_TRACKER_H
