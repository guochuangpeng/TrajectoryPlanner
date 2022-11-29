此功能包的效果展示：
(1).下载功能包pathing_get到工作空间下
(2).配置好PX4、mavros环境（可以根据XTDrone配置）
(3).将/PX4_Firmware/launch文件夹中的mavros_posix_sitl.launch替换为下载文件夹extra中的mavros_posix_sitl.launch
(4).在/PX4_Firmware/Tools/sitl_gazebo/models文件夹中加入下载文件夹extra中的iris_fusion和kinect_self文件夹
(5).运行步骤：在PX4_Firmware路径下打开终端运行 roslaunch px4 mavros_posix_sitl.launch 、在工作空间下运行roslaunch pathing_get go.launch
(6).此程序留出了slam建图接口，已经在无人机中配置好了RGBD摄像头与激光雷达，发布的话题名为/camera/depth/image_raw、/camera/depth/points、/laser/scan，可以用于后端建图


1.此功能包通过读取配置文件config.yaml各项参数，采用minimum-jerk算法生成路径，其中：
（1）route定义了需要经过的点
（2）ts定义了每一段的时间
（3）if_PID可以改变控制方式（开环/速度环）

2.get_path类可以读取yaml的各项数据，并通过调用其类方法generate_path通过minimum-jerk方法生成期望路径，如果需要实时发布路径点并规划路径可以更改构造函数，使其初始化时存储值为你需要的路径点

3.ros_node类成员中集成了服务、话题发布订阅的消息，可以通过类方法show将get_path类中的路径展示出来，并将实际运行轨迹展示出来。
