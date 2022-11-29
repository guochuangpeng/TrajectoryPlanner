#include "controller.h"

//实例化时保证误差为0
control::control()
{  
  reset();
}

control::~control() {}

//将yalm配置文件数值读取到闭环参数中
void control::setParam( Eigen::Vector2d _kp_pos_h, 
                        Eigen::Vector2d _kp_vel_h,
                        Eigen::Vector2d _ki_vel_h,
                        Eigen::Vector2d _kd_vel_h,
                        double _kp_pos_z,
                        double _kp_vel_z,
                        double _ki_vel_z,
                        double _kd_vel_z,
                        Eigen::Vector3d _kp_pose)
{
  kp_pos_h = _kp_pos_h;
  kp_vel_h = _kp_vel_h;
  ki_vel_h = _ki_vel_h;
  kd_vel_h = _kd_vel_h;
  kp_pos_z = _kp_pos_z;
  kp_vel_z = _kp_vel_z;
  ki_vel_z = _ki_vel_z;
  kd_vel_z = _kd_vel_z;
  kp_pose = _kp_pose;
}

//限制速度在最大值范围内
void control::limit_linear_vel(Eigen::Vector2d &linear_vel_desire, double max)
{
  double linear_vel_desire_norm = linear_vel_desire.norm();//norm为求二范数

  linear_vel_desire(0, 0) = linear_vel_desire(0, 0) * min(max, linear_vel_desire_norm) / linear_vel_desire_norm;
  linear_vel_desire(1, 0) = linear_vel_desire(1, 0) * min(max, linear_vel_desire_norm) / linear_vel_desire_norm;
}

//PID闭环主要函数
double *control::PID( XYZYaw pos_desire, 
                      Eigen::Vector3d vel_desire, 
                      Eigen::Vector3d acc_desire, 
                      XYZYaw pos_curr, 
                      Eigen::Vector3d linear_vel_curr, 
                      Eigen::Vector3d pose_curr)
{
  //定义位置速度？
  Eigen::Vector3d pose_rate;
  //定义垂直于飞机的力矩
  double thr;
  
  /*--- 水平面方向（包含x、y） ---*/
  //定义水平方向上的误差以及水平方向上的期望线速度
  Eigen::Vector2d error_pos_h, linear_vel_desire_h;
  error_pos_h(0) = pos_desire.x - pos_curr.x;
  error_pos_h(1) = pos_desire.y - pos_curr.y;
  linear_vel_desire_h(0) = kp_pos_h(0) * error_pos_h(0) + vel_desire(0);
  linear_vel_desire_h(1) = kp_pos_h(1) * error_pos_h(1) + vel_desire(1);
  limit_linear_vel(linear_vel_desire_h, max_vel);

  /*--- height ---*/
  double error_pos_z, linear_vel_desire_z;
  error_pos_z = pos_desire.z - pos_curr.z;
  linear_vel_desire_z = kp_pos_z * error_pos_z + vel_desire(2);

  M[0]=linear_vel_desire_h(0);
  M[1]=linear_vel_desire_h(1);
  M[2]=linear_vel_desire_z;
  M[3]= 0;

  return M;
} 

void control::reset()
{
  last_error_vel_h << 0, 0;
  add_error_vel_h << 0, 0;
  last_error_vel_z = 0;
  add_error_vel_z = 0;
}

