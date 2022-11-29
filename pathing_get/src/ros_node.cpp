#include "ros_node.h"

/*此函数为构造函数，用于初始化ros各项工作*/
ros_node::ros_node()
{
//创建任务句柄
ros::NodeHandle nh_private("~");
//订阅里程计话题名为/mavros/odometry/in信息，回调地址为：&ros_node::get_cur_pos
odom_sub = nh_private.subscribe("/mavros/odometry/in", 1, &ros_node::get_cur_pos, this);
//订阅里程计话题名为/mavros/state信息，回调地址为：&ros_node::get_state
state_sub = nh_private.subscribe<mavros_msgs::State>("/mavros/state", 10, &ros_node::get_state,this);
//发布类型为 geometry_msgs::Point 名为：/error_pose_for_plot 的话题
error_pose_publisher = nh_private.advertise<geometry_msgs::Point>("/error_pose_for_plot", 1);
error_vel_publisher = nh_private.advertise<geometry_msgs::Point>("/error_vel_for_plot", 1);
//发布类型为 nav_msgs::Path 名为：/error_pose_for_plot 的话题
path_desire_pub = nh_private.advertise<nav_msgs::Path>("/path_desire", 1);
path_real_pub = nh_private.advertise<nav_msgs::Path>("/path_real", 1);
//发布类型为 geometry_msgs::PoseStamped 名为：/mavros/setpoint_position/local 的话题
local_pos_pub = nh_private.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
local_vel_pub = nh_private.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
//创建一个client，消息类型为：mavros_msgs::CommandBool
arming_client = nh_private.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
set_mode_client = nh_private.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

//定义相对固定的坐标系为世界坐标系
  path_desire.header.frame_id = "world";
  path_real.header.frame_id = "world";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;
  arm_cmd.request.value = true;
  offb_set_mode.request.custom_mode = "OFFBOARD";

//将读取到的值初始化到控制器中
controller1.setParam( Eigen::Vector2d(get_path1.PID_msgs(0), get_path1.PID_msgs(0)),
                        Eigen::Vector2d(get_path1.PID_msgs(1), get_path1.PID_msgs(1)),
                        Eigen::Vector2d(get_path1.PID_msgs(2), get_path1.PID_msgs(2)),
                        Eigen::Vector2d(get_path1.PID_msgs(3), get_path1.PID_msgs(3)),
                        get_path1.PID_msgs(4),
                        get_path1.PID_msgs(5),
                        get_path1.PID_msgs(6),
                        get_path1.PID_msgs(7),
                        Eigen::Vector3d(get_path1.PID_msgs(8), get_path1.PID_msgs(8), get_path1.PID_msgs(8)));
controller1.max_vel = get_path1.max_vel;

}

//此函数为回调函数，用于读取里程计信息
void ros_node::get_cur_pos(const nav_msgs::Odometry &odom_msg)
{
  //记录当前位置
  curr_position_.x = odom_msg.pose.pose.position.x;
  curr_position_.y = odom_msg.pose.pose.position.y;
  curr_position_.z = odom_msg.pose.pose.position.z;
  //记录线速度
  linear_vel_feedback(0, 0) = odom_msg.twist.twist.linear.x;
  linear_vel_feedback(1, 0) = odom_msg.twist.twist.linear.y;
  linear_vel_feedback(2, 0) = odom_msg.twist.twist.linear.z;
  //将当前位置的x、y、z输入到curr_pos中
  curr_pos << curr_position_.x, curr_position_.y, curr_position_.z;

  //定义存储四元素的变量
  tf::Quaternion quat;
  //odom消息中只有四元数，四元素消息转化为四元素
  tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, quat);
  //四元素不能直接转化为欧拉角，需要借助于一个旋转矩阵
  tf::Matrix3x3 rotation_R;
  //将四元素转化为旋转矩阵
  rotation_R.setRotation(quat);
  //定义存储r\p\y的容器
  double roll, pitch, yaw;
  //利用成员函数转化为欧拉角                      
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); 

  //四元素转旋转矩阵
  Eigen::Quaterniond eigen_quat;
  eigen_quat.x() = odom_msg.pose.pose.orientation.x;
  eigen_quat.y() = odom_msg.pose.pose.orientation.y;
  eigen_quat.z() = odom_msg.pose.pose.orientation.z;
  eigen_quat.w() = odom_msg.pose.pose.orientation.w;
  curr_rotation_matrix = eigen_quat.normalized().toRotationMatrix();
  //将z轴的旋转量记录下来 
  curr_position_.yaw = yaw;
  //用欧拉角记录当前姿态，curr_pose为欧拉角
  curr_pose << roll, pitch, yaw;
}

//此函数为回调函数，用于更新状态信息？
void ros_node::get_state(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

//此函数循环调用显示界面
void ros_node::show()
{
  //定义函数的循环频率为100Hz
  ros::Rate loop_rate(50);

  //更新当前时间
  ros::Time last_request = ros::Time::now();

  //进入大循环
  while (ros::ok())
  { 
    show_path();
    loop_rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char **argv)
{
  //初始化节点
  ros::init(argc, argv, "open_loop");
  //创建实例化对象
  ros_node ros_node1;
  //多线程构造
  std::thread thrd_1(&ros_node::show, &ros_node1);
  thrd_1.join();
}
