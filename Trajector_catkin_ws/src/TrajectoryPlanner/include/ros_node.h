#include "config.h"
#include "camera_set.h"
#include "controller.h"
#include "filter.h"
#include "root_finder.hpp"
#include "traj_min_jerk.hpp"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "planner.h"

class ros_node
{
private:
    ros::Subscriber odom_sub;
    ros::Publisher airsim_anglerate_frame_pub_;
    ros::ServiceClient client_takeoff;
    ros::ServiceClient client_land;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *left_sub; // topic2 输入
    message_filters::Subscriber<sensor_msgs::Image> *right_sub;
    message_filters::Synchronizer<sync_pol> *sync;

    camera_set camera_set1;
    control controller1;
    filter filter1, filter2, filter3;
    double *M = new double[4];

    ros::Publisher error_pose_publisher;
    ros::Publisher error_vel_publisher;
    ros::Publisher path_desire_pub;
    ros::Publisher path_real_pub;
    min_jerk::Trajectory minJerkTraj;

    double max_vel, max_acc;
    bool set_param_flag;
    Eigen::MatrixXd route_ref, route;
    Eigen::Vector3d debug_pos_desire;
    Eigen::VectorXd ts;
    double delay;
    int circle_map[7] = {1, 2, 3, 4, 6, 7, 8};
    // path_planner planner;

public:
    XYZYaw curr_position_;
    XYZYaw expect_position_;
    Eigen::Vector3d linear_vel_feedback;
    Eigen::Vector3d curr_pose, curr_pos, circle_pos;
    Eigen::Matrix3d curr_rotation_matrix;
    Eigen::Vector4d rpyt;

public:
    ros_node();
    void odom_cb(const nav_msgs::Odometry &odom_msg);
    void circle_though();
    void run();
    void get_path();
    void go_to(Eigen::Vector3d pos_desire, Eigen::Vector3d vel_desire, Eigen::Vector3d acc_desire, double yaw_desire);
};
