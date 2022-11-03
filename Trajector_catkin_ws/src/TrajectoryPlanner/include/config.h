
#ifndef CONFIG_H
#define CONFIG_H


#include "ros/ros.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include "airsim_ros_pkgs/VelCmd.h"
#include "airsim_ros_pkgs/AngleRateThrottle.h"
#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/Land.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <opencv2/opencv.hpp>
#include <thread>

struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};
using namespace std;
using namespace cv;
#endif // CONFIG_H
