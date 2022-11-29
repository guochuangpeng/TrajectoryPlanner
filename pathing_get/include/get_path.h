#ifndef GET_PATH_H
#define GET_PATH_H

#include "config.h"
#include "controller.h"
#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

class get_path
{
private:

public:
   //创建整数型变量记录中间点的个数
int ptnums;
//创建参数队列读取中间点的信息
XmlRpc::XmlRpcValue param_list;
//最大速度、最大加速度
double max_vel, max_acc;
//参数设置标记位
bool set_param_flag;
//路径点容器
Eigen::MatrixXd route_ref, route;
//期望路径
Eigen::Vector3d debug_pos_desire;
//路径时间
Eigen::VectorXd ts;
//PID参数
Eigen::VectorXd PID_msgs;
//定义delay类型
double delay;
//实例化一个minimum_jerk对象存储路径
min_jerk::Trajectory minJerkTraj;
//实例化一个minimum_snap对象存储路径
min_snap::Trajectory minSnapTraj;
//选择是否用PID
bool if_PID;

public:
//构建析构函数，在创建实例化对象时直接调用
get_path();
//采用minimum_jerk生成轨迹
void generate_path();
//展示配置文件数据
void show_param();
};


#endif // CONFIG_H