#include "config.h"

class control
{
private:
    double ad_error_limit = 8;
    double max_poserate = 1;

    //-- horizon
    Eigen::Vector2d kp_pos_h;
    Eigen::Vector2d kp_vel_h;
    Eigen::Vector2d ki_vel_h;
    Eigen::Vector2d kd_vel_h;
    Eigen::Vector2d last_error_vel_h;
    Eigen::Vector2d add_error_vel_h;

    //-- height
    double linear_vel_desire_z;
    double kp_pos_z;
    double kp_vel_z;
    double ki_vel_z;
    double kd_vel_z;
    double last_error_vel_z;
    double add_error_vel_z;
    
    Eigen::Vector3d kp_pose;
    double *M = new double[4];

public:
    control();
    ~control();
    void setParam( Eigen::Vector2d _kp_pos_h, 
                    Eigen::Vector2d _kp_vel_h,
                    Eigen::Vector2d _ki_vel_h,
                    Eigen::Vector2d _kd_vel_h,
                    double _kp_pos_z,
                    double _kp_vel_z,
                    double _ki_vel_z,
                    double _kd_vel_z,
                    Eigen::Vector3d _kp_pose);
    void limit_linear_vel(Eigen::Vector2d &linear_vel_desire, double max);
    double *PID(XYZYaw pos_desire,
                Eigen::Vector3d vel_desire,
                Eigen::Vector3d acc_desire,
                XYZYaw pos_curr,
                Eigen::Vector3d linear_vel_curr,
                Eigen::Vector3d pose_curr);
    void reset();

    double max_vel;
    Eigen::Vector2d error_vel_h;
    double error_vel_z;
};
