#include "planer.h"


//读取里程计信息
void planer::odom_cb(const nav_msgs::Odometry &odom_msg)
{
     //记录当前位置
    curr_position_.x = odom_msg.pose.pose.position.x;
    curr_position_.y = odom_msg.pose.pose.position.y;
    curr_position_.z = odom_msg.pose.pose.position.z;
    //记录线速度
    linear_vel_feedback(0) = odom_msg.twist.twist.linear.x;
    linear_vel_feedback(1) = odom_msg.twist.twist.linear.y;
    linear_vel_feedback(2) = odom_msg.twist.twist.linear.z;

    // ROS_INFO("POSITION: x = %.2f  ,y=%.2f  ,z=%.2f",curr_position_.x,curr_position_.y,curr_position_.z);
    // ROS_INFO("VELOCITY: v_x = %.2f  ,v_y=%.2f  ,v_z=%.2f", linear_vel_feedback(0),linear_vel_feedback(1),linear_vel_feedback(2));

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



//更新状态信息
void planer::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}



//设置参数值
void planer::setParam( Eigen::Vector2d _kp_pos_h, 
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



//初始化操作
planer::planer(){
    ros::NodeHandle nh_private("~");
    odom_sub = nh_private.subscribe("/mavros/odometry/in", 1, &planer::odom_cb, this);
    state_sub = nh_private.subscribe<mavros_msgs::State>("/mavros/state", 10, &planer::state_cb,this);
    //发布期望位置和真实位置
    path_desire_pub = nh_private.advertise<nav_msgs::Path>("/path_desire", 1);
    path_real_pub = nh_private.advertise<nav_msgs::Path>("/path_real", 1);
   local_pos_pub = nh_private.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
   local_vel_pub = nh_private.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    //解锁和设置模式
    arming_client = nh_private.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh_private.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    PositionTarget_pub = nh_private.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    //创建ptnums记录点的个数(xyz)
    int ptnums;
    //创建数组读取route的信息
    XmlRpc::XmlRpcValue param_list;
    //判断参数配置文件中是否具有点信息
    if (!nh_private.getParam("route", param_list)) {
      ROS_ERROR("Failed to load parameter from servel.");
      }
    ptnums = param_list.size() / 3;
    //用route_ref、route记录点的数据，.resize用来初始化容器大小，初始化为 3 行  ptnums 列
    route_ref.resize(3, ptnums);
    route.resize(3, ptnums);

    //不断读取并将中间点的内容，存入route_ref中
    for (int i = 0; i < param_list.size(); i++) {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        //按照一列为xyz的插入
        route_ref(i/ptnums, i%ptnums) = double(tmp_value);
      }
    }
    //将值传给route
    route = route_ref;
    //判断配置文件中的 ts 参数列表是否为空
    if (!nh_private.getParam("ts", param_list)) {
      ROS_ERROR("Failed to load parameter from servel.");
    }
    //初始化时间分配值ts容器大小，记录ts数据      数据为 ptnums-1 行  1列  向量
    ts.resize(ptnums-1);
    //往容器里面提取参数列表数据
    for (int i = 0; i < param_list.size(); i++) {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        ts(i) = double(tmp_value);
      }
    }
  //创建k、p、i参数变量，并通过yaml文件读取参数
  double kp_pos_h, kp_vel_h, ki_vel_h, kd_vel_h;
  double kp_pos_z, kp_vel_z, ki_vel_z, kd_vel_z;
  double kp_pose;
    //读取控制器参数
      nh_private.getParam("delay", delay);
      ts *= delay;
      nh_private.getParam("max_vel", max_vel);
      nh_private.getParam("max_acc", max_acc);
      nh_private.getParam("kp_pos_h", kp_pos_h);
      nh_private.getParam("kp_vel_h", kp_vel_h);
      nh_private.getParam("ki_vel_h", ki_vel_h);
      nh_private.getParam("kd_vel_h", kd_vel_h);
      nh_private.getParam("kp_pos_z", kp_pos_z);
      nh_private.getParam("kp_vel_z", kp_vel_z);
      nh_private.getParam("ki_vel_z", ki_vel_z);
      nh_private.getParam("kd_vel_z", kd_vel_z);
      nh_private.getParam("kp_pose", kp_pose);
      setParam(Eigen::Vector2d(kp_pos_h, kp_pos_h),
                            Eigen::Vector2d(kp_vel_h, kp_vel_h),
                            Eigen::Vector2d(ki_vel_h, ki_vel_h),
                            Eigen::Vector2d(kd_vel_h, kd_vel_h),
                            kp_pos_z,
                            kp_vel_z,
                            ki_vel_z,
                            kd_vel_z,
                            Eigen::Vector3d(kp_pose, kp_pose, kp_pose));
     max_vel = max_vel;
      get_path();
}



//用于路径规划
void planer::get_path(){
      min_jerk::JerkOpt jerkOpt;
      Eigen::Matrix3d iS, fS;

      iS.setZero();
      fS.setZero();
      //col表示列
      iS.col(0) << route.leftCols<1>();
      fS.col(0) << route.rightCols<1>();

      jerkOpt.reset(iS, fS, route.cols() - 1);
      //block表示 从route 的第0行 第1列  开始大小为 3 行 route.cols()-2 列的矩阵
      jerkOpt.generate(route.block(0, 1, 3, route.cols() - 2), ts);
      jerkOpt.getTraj(minJerkTraj);
}


//发送真实路径和期望路径
void planer::seed_path(Eigen::Vector3d pos_desire){
        //期望path 
      path_desire.header.stamp = ros::Time::now();
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position.x = pos_desire(0);
      pose.pose.position.y = pos_desire(1);
      pose.pose.position.z = pos_desire(2);
      path_desire.poses.push_back(pose);
      path_desire_pub.publish(path_desire);
      //真实path
      path_real.header.stamp = ros::Time::now();
      pose.pose.position.x = curr_pos(0);
      pose.pose.position.y = curr_pos(1);
      pose.pose.position.z = curr_pos(2);
      path_real.poses.push_back(pose);
      path_real_pub.publish(path_real);
}


//限制速度
void planer::limit_linear_vel(Eigen::Vector2d &linear_vel_desire, double max){
      double linear_vel_desire_norm = linear_vel_desire.norm();
      linear_vel_desire(0, 0) = linear_vel_desire(0, 0) * min(max, linear_vel_desire_norm) / linear_vel_desire_norm;
      linear_vel_desire(1, 0) = linear_vel_desire(1, 0) * min(max, linear_vel_desire_norm) / linear_vel_desire_norm;
}


//速度的闭环控制
double *planer::PID_val(XYZYaw pos_desire, Eigen::Vector3d vel_desire, Eigen::Vector3d acc_desire, XYZYaw pos_curr, Eigen::Vector3d linear_vel_curr, Eigen::Vector3d pose_curr){
      static int count = 0;

      Eigen::Vector2d error_pos_h, linear_vel_desire_h;
      //X Y速度
      error_pos_h(0) = pos_desire.x - pos_curr.x;
      error_pos_h(1) = pos_desire.y - pos_curr.y;
      linear_vel_desire_h(0) = kp_pos_h(0) * error_pos_h(0) + vel_desire(0);
      linear_vel_desire_h(1) = kp_pos_h(1) * error_pos_h(1) + vel_desire(1);
      limit_linear_vel(linear_vel_desire_h,max_vel);
      //Z速度
      double error_pos_z, linear_vel_desire_z;
      error_pos_z = pos_desire.z - pos_curr.z;
      linear_vel_desire_z = kp_pos_z * error_pos_z + vel_desire(2);

      if(count <  800){
        //打印前800次的误差
        ROS_INFO(" C:%d  X: %.2f  Y:%.2f   Z:%.2f  ZP:%.2f",count,error_pos_h(0),error_pos_h(1),error_pos_z,pos_curr.z);
        count++;
      }
      
      M[0]=linear_vel_desire_h(0);
      M[1]=linear_vel_desire_h(1);
      M[2]=linear_vel_desire_z;
     return M;
}


//发送速度控制
void planer::pid_controller(){
    geometry_msgs::TwistStamped PP;
    PP.twist.linear.x = M[0];
    PP.twist.linear.y = M[1];
    PP.twist.linear.z = M[2];
    local_vel_pub.publish(PP);

}


//此函数给的都是期望  调用px4内置的函数进行控制
void planer::controller(Eigen::Vector3d pos_desire, Eigen::Vector3d vel_desire, Eigen::Vector3d acc_desire, double yaw_desire){
        //controller
      mavros_msgs::PositionTarget poseall;
      poseall.position.x = pos_desire(0);
      poseall.position.y = pos_desire(1);
      poseall.position.z = pos_desire(2);
      poseall.velocity.x = vel_desire(0);
      poseall.velocity.y = vel_desire(1);
      poseall.velocity.z = vel_desire(2);
      poseall.acceleration_or_force.x = acc_desire(0);
      poseall.acceleration_or_force.y = acc_desire(1);
      poseall.acceleration_or_force.z = acc_desire(2);
      poseall.yaw = yaw_desire;
      poseall.coordinate_frame=poseall.FRAME_LOCAL_NED;
      PositionTarget_pub.publish(poseall);
}



//------主函数--------
void planer::go(){
      ros::Rate loop_rate(100);
      //记录时间
      static float timer = 0, total_time;
      //坐标系为世界坐标系
      path_desire.header.frame_id = "world";
      path_real.header.frame_id = "world";

      mavros_msgs::SetMode offb_set_mode;
      offb_set_mode.request.custom_mode = "OFFBOARD";

      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;

      geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;
      
      //更新当前时间
      ros::Time last_request = ros::Time::now();
      ros::Duration du(4);
      while (ros::ok()){ 
        if( current_state.mode != "OFFBOARD"){
          if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.mode_sent){
            //ROS_INFO("Offboard enabled");
          }
          last_request = ros::Time::now();
        }
        else {
          if( !current_state.armed){
            if( arming_client.call(arm_cmd) &&arm_cmd.response.success){
              //ROS_INFO("Vehicle armed");
              arm_flag = 1;
            }
            last_request = ros::Time::now();
          }
        }
        //解锁完成
        if(arm_flag == 1){
          if (timer == 0) {
            total_time = minJerkTraj.getTotalDuration();
            ROS_INFO("total time:%.2fs", total_time);
            //睡眠4s  测试飞机稳定后 
            du.sleep();
          }
          if (timer <= total_time) {
                timer += 0.01;
              pos_desire = minJerkTraj.getPos(timer);
              vel_desire = minJerkTraj.getVel(timer);
              acc_desire = minJerkTraj.getAcc(timer);
              yaw_desire = atan2(vel_desire(1), vel_desire(0));
            
              pos_desire_XYZYaw.x = pos_desire(0);
              pos_desire_XYZYaw.y = pos_desire(1);
              pos_desire_XYZYaw.z = pos_desire(2);
              pos_desire_XYZYaw.yaw = yaw_desire;

              seed_path(pos_desire);

              M = PID_val(pos_desire_XYZYaw, vel_desire, acc_desire, curr_position_, linear_vel_feedback, curr_pose);

              pid_controller();

            // controller(pos_desire,vel_desire,acc_desire,yaw_desire);
        }
        }
        else if(arm_flag == 0){
            //发布px4位置   让飞机解锁再控制
            geometry_msgs::PoseStamped pose;
              pose.pose.position.x = 0;
              pose.pose.position.y = 0;
              pose.pose.position.z = 2;
              local_pos_pub.publish(pose);
          }
        loop_rate.sleep();
        ros::spinOnce();
      }
}