#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "config.h"

class control
{
private:
    //累加误差值，设定限制值？
    double ad_error_limit = 8;
    //定义最大姿态速度
    double max_poserate = 1;
    //水平方向上的参数设定
    //水平方向上位置环P值
    Eigen::Vector2d kp_pos_h;
    //水平方向上的速度PID参数
    Eigen::Vector2d kp_vel_h;
    Eigen::Vector2d ki_vel_h;
    Eigen::Vector2d kd_vel_h;
    //水平方向上一时刻的速度误差值
    Eigen::Vector2d last_error_vel_h;
    //水平方向上的速度累计误差值
    Eigen::Vector2d add_error_vel_h;

    //垂直方向上的期望线速度
    double linear_vel_desire_z;
    //垂直方向上的位置环P值
    double kp_pos_z;
    //垂直方向上的速度环PID值
    double kp_vel_z;
    double ki_vel_z;
    double kd_vel_z;
    //垂直方向上一时刻的速度误差值
    double last_error_vel_z;
    //垂直方向上的速度累计误差值
    double add_error_vel_z;
    
    //位置环P值
    Eigen::Vector3d kp_pose;
    //定义指向于浮点数组的地址变量M，分别存储为三个方向角速度与垂直于无人机的力矩
    double *M = new double[4];
    

public:

    //定义最大速度
    double max_vel;
    //定义水平方向上的最大速度误差
    Eigen::Vector2d error_vel_h;
    //定义垂直方向上的速度误差值
    double error_vel_z;

public:
    //构造函数与析构函数
    control();
    ~control();

    //将yalm文件中读取的配置参数加载到变量中
    void setParam( Eigen::Vector2d _kp_pos_h, 
                    Eigen::Vector2d _kp_vel_h,
                    Eigen::Vector2d _ki_vel_h,
                    Eigen::Vector2d _kd_vel_h,
                    double _kp_pos_z,
                    double _kp_vel_z,
                    double _ki_vel_z,
                    double _kd_vel_z,
                    Eigen::Vector3d _kp_pose);

    //防止线速度超过限定的最大值
    //参数1：线速度期望值、线速度限制最大值
    void limit_linear_vel(Eigen::Vector2d &linear_vel_desire, double max);

    //计算闭环输出的M
    //输入参数为：位置期望值、速度期望值、加速度期望值、当前位置、当前速度、当前加速度
    //输出参数为：M（三个旋转速度+整体力矩）
    double *PID(XYZYaw pos_desire,
                Eigen::Vector3d vel_desire,
                Eigen::Vector3d acc_desire,
                XYZYaw pos_curr,
                Eigen::Vector3d linear_vel_curr,
                Eigen::Vector3d pose_curr);
    
    //将水平方向和垂直方向上的累计误差与上次误差清零
    void reset();

    
};

#endif