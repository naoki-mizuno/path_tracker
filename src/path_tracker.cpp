#include "path_tracker/path_tracker.h"

using Pose = PathTracker::Pose;

PathTracker::PathTracker()
    : path{}
    , path_msg{}
    , tf_listener{tf_buffer}
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_p{"~"};
    path_sub = nh.subscribe("path",
                            100,
                            &PathTracker::path_callback,
                            this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    nh_p.param("map_frame", map_frame, std::string{"map"});
    nh_p.param("base_frame", base_frame, std::string{"base_link"});
    nh_p.param("close_enough", close_enough, 2.0);
    nh_p.param("update_rate", update_rate, 10.0);
    nh_p.param("K_d_t", K_d_t, 3.0);
    nh_p.param("K_w", K_w, 1.0);
    nh_p.param("K_d_d", K_d_d, 3.0);
    // Velocity (m/s)
    nh_p.param("v", v, 1.0);
}

PathTracker::PathTracker(const PathTracker& other)
    : path{other.path}
    , path_msg{other.path_msg}
    , path_sub{other.path_sub}
    , cmd_vel_pub{other.cmd_vel_pub}
    , tf_buffer{}
    , tf_listener{tf_buffer}
    , map_frame{other.map_frame}
    , base_frame{other.base_frame}
    , K_d_t{other.K_d_t}
    , K_w{other.K_w}
    , K_d_d{other.K_d_d}
    , close_enough{other.close_enough}
    , update_rate{other.update_rate}
    , v{other.v}
    , w{other.w}
    , d_d{other.d_d}
    , d_t{other.d_t}
{
}

PathTracker::PathTracker(PathTracker&& other) noexcept
    : path{std::move(other.path)}
    , path_msg{std::move(other.path_msg)}
    , path_sub{std::move(other.path_sub)}
    , cmd_vel_pub{std::move(other.cmd_vel_pub)}
    , tf_buffer{}
    , tf_listener{tf_buffer}
    , map_frame{std::move(other.map_frame)}
    , base_frame{std::move(other.base_frame)}
    , K_d_t{std::move(other.K_d_t)}
    , K_w{std::move(other.K_w)}
    , K_d_d{std::move(other.K_d_d)}
    , close_enough{std::move(other.close_enough)}
    , update_rate{std::move(other.update_rate)}
    , v{std::move(other.v)}
    , w{std::move(other.w)}
    , d_d{std::move(other.d_d)}
    , d_t{std::move(other.d_t)}
{
}

PathTracker::~PathTracker()
{
}

std::tuple<double, double, geometry_msgs::Point>
PathTracker::get_v_w(const Pose& pr, const Pose& p0, const Pose& p1) {
    auto p_r = pr.position;
    auto p_0 = p0.position;
    auto p_1 = p1.position;

    // Angle between pr, p0, and p1
    auto ang_r01 = atan2(p_r.y - p_0.y, p_r.x - p_0.x)
                   - atan2(p_1.y - p_0.y, p_1.x - p_0.x);
    // Distance between the robot and p0
    auto dist_r0 = get_dist(p_r, p_0);
    // Angle between p1, p0, and the x axis
    auto ang_10x = atan2(p_1.y - p_0.y, p_1.x - p_0.x);

    // Point on path closest to the robot (p_r)
    auto p_path = geometry_msgs::Point{};
    p_path.x = p_0.x + dist_r0 * cos(ang_r01) * cos(ang_10x);
    p_path.y = p_0.y + dist_r0 * cos(ang_r01) * sin(ang_10x);

    // Find the distance between the robot and the segment
    d_d = dist_r0 * sin(ang_r01);
    // Find the angle between the robot and the segment
    tf2::Quaternion q;
    tf2::fromMsg(pr.orientation, q);
    tf2::Matrix3x3 m{q};
    double roll_r, pitch_r, yaw_r;
    m.getRPY(roll_r, pitch_r, yaw_r);
    d_t = yaw_r - atan2(p_1.y - p_0.y, p_1.x - p_0.x);

    // Limit to [-pi, pi]
    d_t = std::remainder(d_t, 2 * M_PI);

    w -= K_d_t * d_t + K_w * w + K_d_d * d_d;

    return std::make_tuple(v, w, p_path);
}

void
PathTracker::start_control_loop() {
    ros::Rate r{update_rate};
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();

        if (path.size() < 2) {
            continue;
        }

        auto p0 = path[0];
        auto p1 = path[1];
        // Robot
        auto pr = get_robot_pose();
        // Closest point on path
        auto pc = geometry_msgs::Point{};
        std::tie(v, w, pc) = get_v_w(pr, p0, p1);

        if (get_dist(p1.position, pc) < close_enough) {
            ROS_INFO("Robot projection close to checkpoint");
            ROS_INFO_STREAM(path.size() << " points left");
            path.erase(path.begin());
            continue;
        }
        if (get_dist(p0.position, p1.position) < close_enough) {
            ROS_INFO("p0 and p1 too close!");
            ROS_INFO_STREAM(path.size() << " points left");
            path.erase(path.begin());
            continue;
        }

        geometry_msgs::Twist msg;
        msg.linear.x = v;
        msg.angular.z = w;
        cmd_vel_pub.publish(msg);
    }
}

void
PathTracker::path_callback(const nav_msgs::Path& msg) {
    path_msg = msg;

    // Clear all poses
    path.clear();
    for (const auto& p : msg.poses) {
        path.push_back(p.pose);
    }
}

Pose
PathTracker::get_robot_pose() {
    Pose current_pose;

    geometry_msgs::TransformStamped t;
    if (tf_buffer.canTransform(map_frame, base_frame, ros::Time(0))) {
        t = tf_buffer.lookupTransform(map_frame, base_frame, ros::Time(0));
    }

    current_pose.position.x = t.transform.translation.x;
    current_pose.position.y = t.transform.translation.y;
    current_pose.position.z = t.transform.translation.z;
    current_pose.orientation = t.transform.rotation;

    return current_pose;
}

template <typename T>
double
PathTracker::get_dist(const T& a, const T& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
