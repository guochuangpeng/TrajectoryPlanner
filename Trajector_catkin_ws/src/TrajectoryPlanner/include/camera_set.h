#include "config.h"

class camera_set
{
private:
    // 相机内参
    double cx = 320.0;
    double cy = 240.0;
    double fx = 320;
    double fy = 320;
    double baseline = 0.095; //单位是mm
    int g_nHm = 9;
    cv::Mat imLeft_out, imRight_out;
    cv::Mat imLeft_raw, imRight_raw;
    cv::Mat imLeft_tmp, imRight_tmp;
    vector<Vec3f> circles_right;
    vector<Vec3f> circles_left;

public:
    Eigen::Vector3d result;
    bool update_flag = 0;

    double get_depth(double xl, double xr);
    Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth);
    Eigen::Vector3d camera2drone(const Eigen::Vector3d &p_p,
                                 const Eigen::Vector3d &p_drone);
    Eigen::Vector3d pixel2drone(const Eigen::Vector2d &p_p,
                                double depth,
                                const Eigen::Vector3d &p_drone);
    Eigen::Vector3d pixel2world(const Eigen::Vector2d &p_p,
                                double depth,
                                const Eigen::Vector3d &p_drone_world,
                                Eigen::Matrix3d matrix_rotation);
    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft,
                    const sensor_msgs::ImageConstPtr &msgRight);

};
