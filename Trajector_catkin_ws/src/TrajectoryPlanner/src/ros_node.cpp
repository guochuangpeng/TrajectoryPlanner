#include "ros_node.h"

ros_node::ros_node()
{
  ros::NodeHandle nh_private("~");
  odom_sub = nh_private.subscribe("/mavros/odometry/in", 1, &ros_node::odom_cb, this);
  //airsim_anglerate_frame_pub_ = nh_private.advertise<airsim_ros_pkgs::AngleRateThrottle>("/airsim_node/drone_1/angle_rate_throttle_frame", 1);
  error_pose_publisher = nh_private.advertise<geometry_msgs::Point>("/error_pose_for_plot", 1);
  error_vel_publisher = nh_private.advertise<geometry_msgs::Point>("/error_vel_for_plot", 1);
  path_desire_pub = nh_private.advertise<nav_msgs::Path>("/path_desire", 1);
  path_real_pub = nh_private.advertise<nav_msgs::Path>("/path_real", 1);
  local_pos_pub = nh_private.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);

  arming_client = nh_private.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
  set_mode_client = nh_private.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
  state_sub = nh_private.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, &ros_node::state_cb,this);
 // client_takeoff = nh_private.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
  // client_land = nh_private.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/land");

  //left_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_private, "/airsim_node/drone_1/front_left/Scene", 1);
  //right_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_private, "/airsim_node/drone_1/front_right/Scene", 1);
 // sync = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub, *right_sub);
 // sync->registerCallback(boost::bind(&camera_set::GrabStereo, &camera_set1, _1, _2));

  // get param
  double kp_pos_h, kp_vel_h, ki_vel_h, kd_vel_h;
  double kp_pos_z, kp_vel_z, ki_vel_z, kd_vel_z;
  double kp_pose;
  int ptnums;
  XmlRpc::XmlRpcValue param_list;
  if (!nh_private.getParam("route", param_list)) {
    ROS_ERROR("Failed to load parameter from servel.");
  }
  ptnums = param_list.size() / 3;
  route_ref.resize(3, ptnums);
  route.resize(3, ptnums);
  for (int i = 0; i < param_list.size(); i++) {
    XmlRpc::XmlRpcValue tmp_value = param_list[i];
    if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      route_ref(i/ptnums, i%ptnums) = double(tmp_value);
    }
  }
  route = route_ref;
  if (!nh_private.getParam("ts", param_list)) {
    ROS_ERROR("Failed to load parameter from servel.");
  }
  ts.resize(ptnums-1);
  for (int i = 0; i < param_list.size(); i++) {
    XmlRpc::XmlRpcValue tmp_value = param_list[i];
    if (tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      ts(i) = double(tmp_value);
    }
  }
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
  nh_private.getParam("set_param_flag", set_param_flag);
  nh_private.getParam("debug_pos_x", debug_pos_desire(0));
  nh_private.getParam("debug_pos_y", debug_pos_desire(1));
  nh_private.getParam("debug_pos_z", debug_pos_desire(2));
  controller1.setParam( Eigen::Vector2d(kp_pos_h, kp_pos_h),
                        Eigen::Vector2d(kp_vel_h, kp_vel_h),
                        Eigen::Vector2d(ki_vel_h, ki_vel_h),
                        Eigen::Vector2d(kd_vel_h, kd_vel_h),
                        kp_pos_z,
                        kp_vel_z,
                        ki_vel_z,
                        kd_vel_z,
                        Eigen::Vector3d(kp_pose, kp_pose, kp_pose));
  controller1.max_vel = max_vel;
  
  get_path();
}

inline Eigen::VectorXd allocateTime(const Eigen::MatrixXd &wayPs, double vel, double acc) {
    int N = (int)(wayPs.cols()) - 1;
    Eigen::VectorXd durations(N);
    if (N > 0)
    {
        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }
    return durations;
}

void ros_node::get_path() {
  min_jerk::JerkOpt jerkOpt;
  Eigen::Matrix3d iS, fS;

  iS.setZero();
  fS.setZero();
  iS.col(0) << route.leftCols<1>();
  fS.col(0) << route.rightCols<1>();
  // ts = allocateTime(route, max_vel, max_acc);
  // std::cout << "ts:" << ts << std::endl;

  jerkOpt.reset(iS, fS, route.cols() - 1);
  jerkOpt.generate(route.block(0, 1, 3, route.cols() - 2), ts);
  jerkOpt.getTraj(minJerkTraj);
}

void ros_node::odom_cb(const nav_msgs::Odometry &odom_msg)
{

  curr_position_.x = odom_msg.pose.pose.position.x;
  curr_position_.y = odom_msg.pose.pose.position.y;
  curr_position_.z = odom_msg.pose.pose.position.z;
  linear_vel_feedback(0, 0) = odom_msg.twist.twist.linear.x;
  linear_vel_feedback(1, 0) = odom_msg.twist.twist.linear.y;
  linear_vel_feedback(2, 0) = odom_msg.twist.twist.linear.z;

  curr_pos << curr_position_.x, curr_position_.y, curr_position_.z;
  tf::Quaternion quat;

  tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, quat);

  tf::Matrix3x3 rotation_R;
  rotation_R.setRotation(quat);

  Eigen::Quaterniond eigen_quat;
  eigen_quat.x() = odom_msg.pose.pose.orientation.x;
  eigen_quat.y() = odom_msg.pose.pose.orientation.y;
  eigen_quat.z() = odom_msg.pose.pose.orientation.z;
  eigen_quat.w() = odom_msg.pose.pose.orientation.w;
  curr_rotation_matrix = eigen_quat.normalized().toRotationMatrix();
  // curr_rotation_matrix<<rotation_R(0,0),rotation_R[0,1],rotation_R[0,2],
  //                       rotation_R[1,0],rotation_R[1,1],rotation_R[1,2],
  //                       rotation_R[2,0],rotation_R[2,1],rotation_R[2,2];
  double roll, pitch, yaw;                      //定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换

  curr_position_.yaw = yaw;
  curr_pose << roll, pitch, yaw;
}

void ros_node::circle_though()
{
  ros::Rate loop_rate(20); // 20Hz
  Eigen::Vector3d single_piexl;
  Eigen::Vector3d circle_pos_get;
  Eigen::Vector2d p_p;
  
  while (ros::ok()) {
    if (camera_set1.update_flag) {
      camera_set1.update_flag = 0;
      single_piexl = camera_set1.result;
      p_p << single_piexl(0), single_piexl(1);
      circle_pos_get = camera_set1.pixel2world(p_p, single_piexl(2), curr_pos, curr_rotation_matrix);
      for (int i = 0; i < 7; i++) {
        int key = circle_map[i];
        if (fabs(circle_pos_get(0)-route_ref(0,key))<1.0 && fabs(circle_pos_get(1)-route_ref(1,key))<1.0 && fabs(circle_pos_get(2)-route_ref(2,key))<1.0) {
          // reset
          if (fabs(circle_pos_get(0)-circle_pos(0))>10.0 || fabs(circle_pos_get(1)-circle_pos(1))>10.0 || fabs(circle_pos_get(2)-circle_pos(2))>10.0) {
            filter1.reset();
            filter2.reset();
            filter3.reset();
            ROS_INFO("Filter reset.");
          }
          // filter
          circle_pos(0) = filter1.add_filter(circle_pos_get(0));
          circle_pos(1) = filter2.add_filter(circle_pos_get(1));
          circle_pos(2) = filter3.add_filter(circle_pos_get(2));
          // circle_pos(0) = (circle_pos_get(0));
          // circle_pos(1) = (circle_pos_get(1));
          // circle_pos(2) = (circle_pos_get(2));

          if (fabs(circle_pos(0)-route_ref(0,key))<1.0 && fabs(circle_pos(1)-route_ref(1,key))<1.0 && fabs(circle_pos(2)-route_ref(2,key))<1.0) { // 1m error
            if (fabs(circle_pos(0)-route(0,key))>0.15 || fabs(circle_pos(1)-route(1,key))>0.15 || fabs(circle_pos(2)-route(2,key))>0.15) {
              // update
              route(0, key) = circle_pos(0);
              route(1, key) = circle_pos(1);
              route(2, key) = circle_pos(2);
              get_path();
              cout << "Position of circle " << i+1 << "/7 is updated:" << endl << circle_pos << endl;
            }
          }
        }
      }
    }
    
    loop_rate.sleep();
    ros::spinOnce();
  }
}

void ros_node::run()
{
  ros::Rate loop_rate(100); // 100Hz
  Eigen::Vector3d pos_desire, vel_desire, acc_desire;
  static float timer = 0, total_time;
  float yaw_desire;

  // airsim_ros_pkgs::Takeoff takeoff;
  // client_takeoff.call(takeoff);

  nav_msgs::Path path_desire, path_real;
  path_desire.header.frame_id = "world";
  path_real.header.frame_id = "world";

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  ros::Time last_request = ros::Time::now();

  geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
        
    // }

  while (ros::ok())
  { 
    if( current_state.mode != "OFFBOARD"){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                //ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                   // ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

  

    if (set_param_flag) {
      go_to(debug_pos_desire, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0.0);
      // plot
      geometry_msgs::Point error;
      error.x = curr_pos(0) - debug_pos_desire(0);
      error.y = curr_pos(1) - debug_pos_desire(1);
      error.z = curr_pos(2) - debug_pos_desire(2);
      error_pose_publisher.publish(error);
      error.x = controller1.error_vel_h(0);
      error.y = controller1.error_vel_h(1);
      error.z = controller1.error_vel_z;
      error_vel_publisher.publish(error);

      
    } 
    else {
      if (timer == 0) {
          total_time = minJerkTraj.getTotalDuration();
          ROS_INFO("total time:%.2fs", total_time);
      }
      if (timer <= total_time) {
          timer += 0.01;
        
          pos_desire = minJerkTraj.getPos(timer);
          vel_desire = minJerkTraj.getVel(timer);
          acc_desire = minJerkTraj.getAcc(timer);
          yaw_desire = atan2(vel_desire(1), vel_desire(0));
          go_to(pos_desire, vel_desire, acc_desire, yaw_desire);
          
          // plot
          geometry_msgs::Point error;
          error.x = curr_pos(0) - pos_desire(0);
          error.y = curr_pos(1) - pos_desire(1);
          error.z = curr_pos(2) - pos_desire(2);
          error_pose_publisher.publish(error);
          error.x = linear_vel_feedback(0) - vel_desire(0);
          error.y = linear_vel_feedback(1) - vel_desire(1);
          error.z = linear_vel_feedback(2) - vel_desire(2);
          error_vel_publisher.publish(error);

          // path 
          path_desire.header.stamp = ros::Time::now();
          path_real.header.stamp = ros::Time::now();
          geometry_msgs::PoseStamped pose;
          pose.header.frame_id = "world";
          pose.pose.position.x = pos_desire(0);
          pose.pose.position.y = pos_desire(1);
          pose.pose.position.z = pos_desire(2);
          path_desire.poses.push_back(pose);
          path_desire_pub.publish(path_desire);
          pose.pose.position.x = curr_pos(0);
          pose.pose.position.y = curr_pos(1);
          pose.pose.position.z = curr_pos(2);
          path_real.poses.push_back(pose);
          path_real_pub.publish(path_real);

          pose.pose.position.x = pos_desire(0);
          pose.pose.position.y = pos_desire(1);
          pose.pose.position.z = pos_desire(2);
           local_pos_pub.publish(pose);

          
      } else {
          go_to(pos_desire, vel_desire, acc_desire, yaw_desire);
      }
    }

    loop_rate.sleep();
    ros::spinOnce();
  }
}

void ros_node::go_to(Eigen::Vector3d pos_desire, Eigen::Vector3d vel_desire, Eigen::Vector3d acc_desire, double yaw_desire) {
  XYZYaw pos_desire_XYZYaw;
  pos_desire_XYZYaw.x = pos_desire(0);
  pos_desire_XYZYaw.y = pos_desire(1);
  pos_desire_XYZYaw.z = pos_desire(2);
  pos_desire_XYZYaw.yaw = yaw_desire;
  // cal
  M = controller1.PID(pos_desire_XYZYaw, vel_desire, acc_desire, curr_position_, linear_vel_feedback, curr_pose);
  rpyt << M[0], M[1], M[2], M[3];
  //cout<<"rpyt"<<rpyt<<endl;
  // set
//   airsim_ros_pkgs::AngleRateThrottle anglerate;
//   anglerate.rollRate = rpyt(0, 0);
//   anglerate.pitchRate = rpyt(1, 0);
//   anglerate.yawRate = rpyt(2, 0);
//   anglerate.throttle = rpyt(3, 0);
//   airsim_anglerate_frame_pub_.publish(anglerate);
}
void ros_node::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_loop");

  ros_node ros_node1;
  std::thread thrd_1(&ros_node::run, &ros_node1);
  std::thread thrd_2(&ros_node::circle_though, &ros_node1);
  thrd_1.join();
  thrd_2.join();
}

