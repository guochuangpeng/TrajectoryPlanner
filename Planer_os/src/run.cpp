#include "planer.h"

//根据需要改动控制环节
// class Control :public planer{
//     public:
//     void pid_controller();
//     double *PID_val(XYZYaw pos_desire, Eigen::Vector3d vel_desire, Eigen::Vector3d acc_desire, XYZYaw pos_curr, Eigen::Vector3d linear_vel_curr, Eigen::Vector3d pose_curr);
// };



int main(int argc, char **argv)
{
  //初始化节点
  ros::init(argc, argv, "os_planer");
  //创建实例化对象
  planer planer_node;
  //多线程构造
  std::thread thrd_1(&planer::go, &planer_node);


  //  改动完使用下面
  //  Control con1;
  //  std::thread thrd_1(&Control::go, &con1);
  thrd_1.join();
}