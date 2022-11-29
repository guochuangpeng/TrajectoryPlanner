#include "get_path.h"

get_path::get_path()
{
ros::NodeHandle nh_private("~");

//判断路径点
if (!nh_private.getParam("route", param_list)) 
{
  ROS_ERROR("Failed to load parameter of waitpoint from servel.");
}
ptnums = param_list.size() / 3;
//用route_ref、route记录中间点的数据，.resize用来初始化容器大小，数据格式3*个数？
route_ref.resize(3, ptnums);
route.resize(3, ptnums);
//不断读取并将中间点的内容，存入route_ref中
for (int i = 0; i < param_list.size(); i++) 
{
  XmlRpc::XmlRpcValue tmp_value = param_list[i];
  if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) 
  {
    route_ref(i/ptnums, i%ptnums) = double(tmp_value);
  }
}
//将值传给route
route = route_ref;


//判断配置文件中的 ts 参数列表是否为空
if (!nh_private.getParam("ts", param_list)) 
{
  ROS_ERROR("Failed to load parameter of ts from servel.");
}
//初始化时间分配值ts容器大小，记录ts数据
ts.resize(ptnums-1);
//往容器里面提取参数列表数据
for (int i = 0; i < param_list.size(); i++) 
{
  XmlRpc::XmlRpcValue tmp_value = param_list[i];
  if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) 
  {
    ts(i) = double(tmp_value);
  }
}

//判断PID参数列表是否为空
if (!nh_private.getParam("PID_msgs", param_list)) 
{
  ROS_ERROR("Failed to load parameter of PID from servel.");
}
//初始化PID参数数组大小
PID_msgs.resize(ptnums-1);
//往容器里面提取参数列表数据
for (int i = 0; i < param_list.size(); i++) 
{
  XmlRpc::XmlRpcValue tmp_value = param_list[i];
  if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) 
  {
    PID_msgs(i) = double(tmp_value);
  }
}

//if_PID为闭环参数标志位1为开启闭环控制
nh_private.getParam("if_PID", if_PID);
//delay为时间分配放大值
nh_private.getParam("delay", delay);
ts *= delay;
//最大速度与加速度限制
nh_private.getParam("max_vel", max_vel);
nh_private.getParam("max_acc", max_acc);
//读取位置存在debug_pos_desire中
nh_private.getParam("debug_pos_x", debug_pos_desire(0));
nh_private.getParam("debug_pos_y", debug_pos_desire(1));
nh_private.getParam("debug_pos_z", debug_pos_desire(2));

//此函数用于展示参数读取结果
show_param();

//采用minimum_jerk生成轨迹
generate_path();

}

//生成轨迹
void get_path::generate_path() 
{
  min_jerk::JerkOpt jerkOpt;
  Eigen::Matrix3d iS, fS;

  iS.setZero();
  fS.setZero();
  iS.col(0) << route.leftCols<1>();
  fS.col(0) << route.rightCols<1>();

  //采用minimum_jerk生成轨迹
  jerkOpt.reset(iS, fS, route.cols() - 1);
  jerkOpt.generate(route.block(0, 1, 3, route.cols() - 2), ts);
  jerkOpt.getTraj(minJerkTraj);
  cout<<"已规划路径，蓝色轨迹为采用minimum—jerk算法所得到的结果"<<endl;

  min_snap::SnapOpt snapOpt;
  Eigen::Matrix<double, 3, 4> IS, FS;

  IS.setZero();
  FS.setZero();
  IS.col(0) << route.leftCols<1>();
  FS.col(0) << route.rightCols<1>();

  //采用minimum_snap生成轨迹
  snapOpt.reset(IS, FS, route.cols() - 1);
  snapOpt.generate(route.block(0, 1, 3, route.cols() - 2), ts);
  snapOpt.getTraj(minSnapTraj);
  cout<<"已规划路径，绿色轨迹为采用minimum—snap算法所得到的结果"<<endl;
}

void get_path::show_param()
{
  using namespace std;
  cout<<"读取配置文件成功"<<endl;
  cout<<"经过的点个数为:"<<endl;
  ROS_INFO("ptnums =  %d ", ptnums);
  
}