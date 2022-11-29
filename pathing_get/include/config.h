//此函数用于包含所用到的相关头文件信息，定义了控制量结构体，调用此文件后，不需要再次声明命名空间
#ifndef CONFIG_H
#define CONFIG_H

#include "ros/ros.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <thread>

struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};
using namespace std;
#endif // CONFIG_H