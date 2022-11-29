#ifndef  _PLANER_H
#define _PLANER_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "traj_min_jerk.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <thread>

#include <mavros_msgs/PositionTarget.h>


struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};
using namespace std;

class planer{
    private:
        ros::Subscriber odom_sub;
        ros::Subscriber state_sub ;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::Publisher local_pos_pub; 
        //px4函数控制
        ros::Publisher PositionTarget_pub;
        //发布期望位置
        ros::Publisher path_desire_pub;
        //发布真实位置
        ros::Publisher path_real_pub;
        min_jerk::Trajectory minJerkTraj;

        //MatrixXd是 大小不确定，数据类型为double的矩阵  用来存储位置信息
        Eigen::MatrixXd route_ref, route;
        //x行的向量  存储时间信息
        Eigen::VectorXd ts;
    
        mavros_msgs::State current_state;
        //延迟
        double delay;
         //判断飞机是否解锁
        int arm_flag = 0;
         //定义存储期望路径、真实路径的变量
         nav_msgs::Path path_desire, path_real;

    public:
        //当前位置
        XYZYaw curr_position_; 
        //当前实际速度
        Eigen::Vector3d linear_vel_feedback;
        //当前实际位置
        Eigen::Vector3d curr_pos,curr_pose;
        Eigen::Matrix3d curr_rotation_matrix;
        //定义期望的yaw角度
        float yaw_desire;
        // 定义期望的位置  速度  加速度
        Eigen::Vector3d pos_desire, vel_desire, acc_desire;


    public:
        /* ----------速度控制-------------*/
        //发布速度消息
        ros::Publisher local_vel_pub;
        //期望位置
        XYZYaw pos_desire_XYZYaw;
        //速度控制的xyz
        double *M = new double[3];
        //定义最大速度  加速度
        double max_vel,max_acc;
        //xy的pid值
        Eigen::Vector2d kp_pos_h,
                                    kp_vel_h,ki_vel_h,kd_vel_h;
        //z的pid值
        double kp_pos_z,
                    kp_vel_z,ki_vel_z,kd_vel_z;
        //xyz P值          
        Eigen::Vector3d kp_pose;

    public:
        planer();
        void odom_cb(const nav_msgs::Odometry &odom_msg);
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void get_path();
        void go();
        void seed_path(Eigen::Vector3d pos_desire);
        void controller(Eigen::Vector3d pos_desire, Eigen::Vector3d vel_desire, Eigen::Vector3d acc_desire, double yaw_desire); 
        void setParam( Eigen::Vector2d _kp_pos_h, Eigen::Vector2d _kp_vel_h, Eigen::Vector2d _ki_vel_h,Eigen::Vector2d _kd_vel_h,double _kp_pos_z,double _kp_vel_z,double _ki_vel_z,double _kd_vel_z,Eigen::Vector3d _kp_pose);
        void limit_linear_vel(Eigen::Vector2d &linear_vel_desire, double max);
         //可根据需要重写pid    继承planer类
        virtual double *PID_val( XYZYaw pos_desire, Eigen::Vector3d vel_desire, Eigen::Vector3d acc_desire,  XYZYaw pos_curr, Eigen::Vector3d linear_vel_curr, Eigen::Vector3d pose_curr);
        virtual void pid_controller();
};

#endif