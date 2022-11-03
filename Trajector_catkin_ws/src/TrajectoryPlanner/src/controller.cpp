#include "controller.h"

control::control()
{  
  reset();
}

control::~control() {}

void control::setParam( Eigen::Vector2d _kp_pos_h, 
                        Eigen::Vector2d _kp_vel_h,
                        Eigen::Vector2d _ki_vel_h,
                        Eigen::Vector2d _kd_vel_h,
                        double _kp_pos_z,
                        double _kp_vel_z,
                        double _ki_vel_z,
                        double _kd_vel_z,
                        Eigen::Vector3d _kp_pose){
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

void control::limit_linear_vel(Eigen::Vector2d &linear_vel_desire, double max)
{
  double linear_vel_desire_norm = linear_vel_desire.norm();

  linear_vel_desire(0, 0) = linear_vel_desire(0, 0) * min(max, linear_vel_desire_norm) / linear_vel_desire_norm;
  linear_vel_desire(1, 0) = linear_vel_desire(1, 0) * min(max, linear_vel_desire_norm) / linear_vel_desire_norm;
}

double *control::PID( XYZYaw pos_desire, 
                      Eigen::Vector3d vel_desire, 
                      Eigen::Vector3d acc_desire, 
                      XYZYaw pos_curr, 
                      Eigen::Vector3d linear_vel_curr, 
                      Eigen::Vector3d pose_curr)
{
  Eigen::Vector3d pose_rate;
  double thr;
  
  /*--- horizon ---*/
  // pos -> vel
  Eigen::Vector2d error_pos_h, linear_vel_desire_h;
  error_pos_h(0) = pos_desire.x - pos_curr.x;
  error_pos_h(1) = pos_desire.y - pos_curr.y;
  linear_vel_desire_h(0) = kp_pos_h(0) * error_pos_h(0) + vel_desire(0);
  linear_vel_desire_h(1) = kp_pos_h(1) * error_pos_h(1) + vel_desire(1);
  limit_linear_vel(linear_vel_desire_h, max_vel);
  // vel -> acc
  Eigen::Vector2d linear_acc_desire_h;
  error_vel_h(0) = linear_vel_desire_h(0) - linear_vel_curr(0);
  error_vel_h(1) = linear_vel_desire_h(1) - linear_vel_curr(1);
  add_error_vel_h = add_error_vel_h + error_vel_h;
  limit_linear_vel(add_error_vel_h, ad_error_limit);
  linear_acc_desire_h(0, 0) = kp_vel_h(0, 0) * error_vel_h(0, 0) + ki_vel_h(0, 0) * add_error_vel_h(0, 0) + kd_vel_h(0, 0) * (error_vel_h(0, 0) - last_error_vel_h(0, 0)) + acc_desire(0);
  linear_acc_desire_h(1, 0) = kp_vel_h(1, 0) * error_vel_h(1, 0) + ki_vel_h(1, 0) * add_error_vel_h(1, 0) + kd_vel_h(1, 0) * (error_vel_h(1, 0) - last_error_vel_h(1, 0)) + acc_desire(1);
  last_error_vel_h = error_vel_h;
  // acc -> angle
  double yaw_curr, roll_desire, pitch_desire;
  yaw_curr = pose_curr(2, 0);
  roll_desire = (-sin(yaw_curr) * linear_acc_desire_h(0) + cos(yaw_curr) * linear_acc_desire_h(1)) / 9.8;
  roll_desire = roll_desire * min(fabs(roll_desire), 1.05) / fabs(roll_desire);
  pitch_desire = (-cos(yaw_curr) * linear_acc_desire_h(0) - sin(yaw_curr) * linear_acc_desire_h(1)) / 9.8;
  pitch_desire = pitch_desire * min(fabs(pitch_desire), 1.05) / fabs(pitch_desire);

  /*--- height ---*/
  // pos -> vel
  double error_pos_z, linear_vel_desire_z;
  error_pos_z = pos_desire.z - pos_curr.z;
  linear_vel_desire_z = kp_pos_z * error_pos_z + vel_desire(2);
  // vel -> acc
  double linear_acc_desire_z;
  error_vel_z = linear_vel_desire_z - linear_vel_curr(2);
  add_error_vel_z = add_error_vel_z + error_vel_z;
  add_error_vel_z = add_error_vel_z * min(fabs(add_error_vel_z), ad_error_limit) / fabs(add_error_vel_z);
  linear_acc_desire_z = kp_vel_z * error_vel_z + ki_vel_z * add_error_vel_z + kd_vel_z * (error_vel_z - last_error_vel_z) + acc_desire(2);
  last_error_vel_z = error_vel_z;
  // acc -> throtle
  thr = -linear_acc_desire_z * min(fabs(linear_acc_desire_z), 0.9) / fabs(linear_acc_desire_z) + 0.5935;

  /*--- get rate ---*/
  pose_rate(0, 0) = kp_pose(0, 0) * (roll_desire - pose_curr(0, 0));
  pose_rate(1, 0) = kp_pose(1, 0) * (pitch_desire - pose_curr(1, 0));
  pose_rate(2, 0) = kp_pose(2, 0) * (pos_desire.yaw - yaw_curr);
  if( pose_rate(2, 0) == 0 ) pose_rate(2, 0) = 0.001;
  // limit
  M[0] = min(fabs(pose_rate(0, 0)), 2.5) * pose_rate(0, 0) / (fabs(pose_rate(0, 0))); // rollrate
  M[1] = min(fabs(pose_rate(1, 0)), 2.5) * pose_rate(1, 0) / (fabs(pose_rate(1, 0))); // pitchrate
  M[2] = min(fabs(pose_rate(2, 0)), 1.5) * pose_rate(2, 0) / (fabs(pose_rate(2, 0))); // yawrate
  M[3] = thr;                                                                         //
  return M;
}

void control::reset()
{
  last_error_vel_h << 0, 0;
  add_error_vel_h << 0, 0;
  last_error_vel_z = 0;
  add_error_vel_z = 0;
}

