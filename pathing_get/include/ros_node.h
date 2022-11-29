#ifndef ROS_NODE_H
#define ROS_NODE_H
#include "config.h"
#include "controller.h"
#include "root_finder.hpp"
#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "get_path.h"

class ros_node
{
private:
    ros::Subscriber odom_sub;
    ros::Subscriber state_sub ;

    ros::ServiceClient client_takeoff;
    ros::ServiceClient client_land;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;

    ros::Publisher local_pos_pub;
    ros::Publisher local_vel_pub;
    ros::Publisher local_acc_pub;
    ros::Publisher error_pose_publisher;
    ros::Publisher error_vel_publisher;
    ros::Publisher path_desire_pub;
    ros::Publisher path_real_pub;

    double *M = new double[4];
    XYZYaw pos_desire_XYZYaw;

    mavros_msgs::State current_state;
    
    // path_planner planner;

public:
    XYZYaw curr_position_;
    XYZYaw expect_position_;
    Eigen::Vector3d linear_vel_feedback;
    Eigen::Vector3d curr_pose, curr_pos;
    Eigen::Matrix3d curr_rotation_matrix;
    Eigen::Vector4d rpyt;
    Eigen::Vector3d vxyz;

    get_path get_path1;
    //实例化一个control对象
    control controller1;
    
    //定义三维向量分别记录位置、速度、加速度
    Eigen::Vector3d pos_desire, vel_desire, acc_desire;
    //定义期望的yaw角度
    float yaw_desire;
    //定义存储期望路径、真实路径的变量
    nav_msgs::Path path_desire, path_real;
    //先实例化一个geometry_msgs::PoseStamped类型的对象，并对其赋值，最后将其发布出去
    geometry_msgs::PoseStamped pose;
    //建立一个类型为CommandBool的服务端arm_cmd，并将其中的是否解锁设为"true"，作用便是用于后面的客户端与服务端之间的通信（服务）
    mavros_msgs::CommandBool arm_cmd;
    //建立一个类型为SetMode的服务端offb_set_mode，并将其中的模式mode设为"OFFBOARD"，作用便是用于后面的客户端与服务端之间的通信（服务）
    mavros_msgs::SetMode offb_set_mode;
    
public:
    ros_node();
    void show();
    void get_cur_pos(const nav_msgs::Odometry &odom_msg);
    void get_state(const mavros_msgs::State::ConstPtr& msg);
    inline void show_path()
    {
        //定义记录时间的变量
            static float timer = 0, total_time_snap, total_time_jerk;
            //首先判断当前模式是否为offboard模式，如果不是，则客户端set_mode_client向服务端offb_set_mode发起请求call，
            //然后服务端回应response将模式返回，这就打开了offboard模式
            if( current_state.mode != "OFFBOARD")
            {
            if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.mode_sent)
            {
                //ROS_INFO("Offboard enabled");
                //打开模式后打印信息
            }
            }
            //else指已经为offboard模式，然后进去判断是否解锁，如果没有解锁，则客户端arming_client向服务端arm_cmd发起请求call
            //然后服务端回应response成功解锁，这就解锁了
            else 
            {
            if( !current_state.armed)
            {
                if( arming_client.call(arm_cmd) &&arm_cmd.response.success)
                {
                // ROS_INFO("Vehicle armed");
                //解锁后打印信息
                }
            }
            }

            if (timer == 0) 
            {
                total_time_snap = get_path1.minSnapTraj.getTotalDuration();
                cout << "采用minimum—snap算法花费的时间为:" << total_time_snap << "s" << endl;
                // ROS_INFO("minimum-snap total time:%.2fs", total_time_snap);

                total_time_jerk = get_path1.minJerkTraj.getTotalDuration();
                cout << "采用minimum—jerk算法花费的时间为:" << total_time_snap << "s" << endl;
                // ROS_INFO("minimum-jerk total time:%.2fs", total_time_jerk);

            }

            //想要使用jerk算法则将下面的minSnapTraj修改为minJerkTraj即可
            if (timer <= total_time_snap) 
            {
                timer += 0.01;
                
                pos_desire = get_path1.minSnapTraj.getPos(timer);
                vel_desire = get_path1.minSnapTraj.getVel(timer);
                acc_desire = get_path1.minSnapTraj.getAcc(timer);
                yaw_desire = atan2(vel_desire(1), vel_desire(0));
                
                pos_desire_XYZYaw.x = pos_desire(0);
                pos_desire_XYZYaw.y = pos_desire(1);
                pos_desire_XYZYaw.z = pos_desire(2);
                pos_desire_XYZYaw.yaw = yaw_desire;
                M = controller1.PID(pos_desire_XYZYaw, vel_desire, acc_desire, curr_position_, linear_vel_feedback, curr_pose);

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

                geometry_msgs::Twist line_vel;

                if (get_path1.if_PID = 0)
                {
                // std::cout<<"开环模式";
                local_pos_pub.publish(pose);
                }
                else
                {
                // std::cout<<"闭环模式"; 
                line_vel.linear.x = M[0];
                line_vel.linear.y = M[1];
                line_vel.linear.z = M[2];
                local_vel_pub.publish(line_vel);
                }
                

            }
    }
};

#endif