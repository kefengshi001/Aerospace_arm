#include <iostream>
#include <vector>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <nav_msgs/Path.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <Eigen/Core>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>



#define SIGINT 2
#define numPoints 10000
bool isRuning = true;
KDL::Tree kdl_tree;
KDL::Chain kdl_chain;

#pragma region // 空间中离散两点之间线段
struct Point3D
{
    double x, y, z;
};

std::vector<Point3D> discretizeLine(const Point3D &start, const Point3D &end)
{
    std::vector<Point3D> points;
   
    double dx = (end.x - start.x) / (numPoints - 1);
    double dy = (end.y - start.y) / (numPoints - 1);
    double dz = (end.z - start.z) / (numPoints - 1);
   
    for (int i = 0; i < numPoints; ++i)
    {
        double x = start.x + i * dx;
        double y = start.y + i * dy;
        double z = start.z + i * dz;


        points.push_back({x, y, z});
    }

    return points;
}
#pragma endregion

#pragma region // 读取键盘Ctrl+c信号并退出
void signalHandler(int signo)
{
    if (signo == SIGINT)
    {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRuning = false;
        std::cout << "isRuning" << isRuning;

        exit(0);
    }
}
#pragma endregion

#pragma region  //显示路径
void publishPath(ros::NodeHandle& nh, const std::string& frame_id, const std::vector<geometry_msgs::PoseStamped>& points) {
    // 创建一个发布者，发布类型为nav_msgs::Path的消息，话题名称为"/path_topic"
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path_topic", 10);

    // 定义要发布的Path消息
    nav_msgs::Path path;
    path.header.frame_id = frame_id;

    // 添加路径上的点
    path.poses = points;

    // 发布Path消息
    path_pub.publish(path);
}
#pragma endregion

//离散点所处直线的两端点
Point3D start_point{-0.644451, 8, 3}, end_point{-0.644451, 8, 0};

int main(int argc, char *argv[])
{
    // Ctrl+c退出
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

#pragma region // KDL解析URDF模型并创建求解器

    // std::string fixed_urdf_file = "/home/k/文档/Aerospace-arm/src/modified-urdf/urdf/modified-urdf-fixed.urdf";
    std::string fixed_urdf_file = "src/my_ros_package/config/modified-urdf-fixed.urdf";


    if (!kdl_parser::treeFromFile(fixed_urdf_file, kdl_tree)) // 建立tree
    {
        // 加载URDF文件失败，处理错误
        std::cout << "加载URDF文件失败" << std::endl;
        return -1;
    }

    // 提取机器人链信息
    std::string base_link = "base_link"; // 设置基准链接
    std::string end_link = "Link7";      // 设置终端链接

    if (!kdl_tree.getChain(base_link, end_link, kdl_chain)) // 建立运动链
    {
        // 获取机器人链失败，处理错误
        std::cout << "获取机器人链失败" << std::endl;
        return -1;
    }

    // 创建正向运动学求解器
    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

    // 创建逆向运动学求解器
    KDL::ChainIkSolverPos_LMA ik_solver(kdl_chain);

    // 创建并初始化关节角
    KDL::JntArray target_joint_positions(kdl_chain.getNrOfJoints());

    target_joint_positions(0) = 45 * KDL::deg2rad;
    target_joint_positions(1) = 90 * KDL::deg2rad;
    target_joint_positions(2) = -45 * KDL::deg2rad;
    target_joint_positions(3) = 0 * KDL::deg2rad;

    // 目标位置
    KDL::Frame pose_start;
    // end_effector_pose.p = KDL::Vector(100,150,100);
    // end_effector_pose.M = KDL::Rotation::RotX(M_PI_2);

    // 正向运动学计算
    int fk_result = fk_solver.JntToCart(target_joint_positions, pose_start);

#pragma endregion

    ros::init(argc, argv, "pub_points_in_line");//末端走直线
    ros::init(argc, argv, "path_publisher");//直线路径显示
    ros::NodeHandle nh;

    ros::Publisher pub_points_in_line = nh.advertise<sensor_msgs::JointState>("/joint_states", 10); //关节
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path_topic", 10);//路径
    ros::Rate rate(1000); // 设置发布频率为100Hz

    sensor_msgs::JointState joint_state;
    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    //路径相关消息类型
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.header.frame_id = "base_link";
    std::vector<geometry_msgs::PoseStamped> points;
 
    // double pose[7]={0, 1, 0.0, 0.0, 0.0, 0.0, 0.0};
    // joint_state.position = {0, 1, 0.0, 0.0, 0.0, 0.0, 0.0}; // 设置初始关节角度

#pragma region // 离散数据点并转化成关节角并发布数据
    std::vector<Point3D> discretizedPoints = discretizeLine(start_point, end_point);
    
    KDL::Frame start_frame;
    KDL::Frame end_frame;
    KDL::Frame point_frame;
    point_frame = start_frame = end_frame = pose_start;
    KDL::JntArray q_init(kdl_chain.getNrOfJoints());
    KDL::JntArray solution(kdl_chain.getNrOfJoints());

    std::cout<<"kdl_chain.getNrOfJoints():"<<kdl_chain.getNrOfJoints()<<std::endl;

    long int count = 0;
    for (int i = 0; i < numPoints; i++)
    {
        //处理joint_state数据
        point_frame.p[0] = discretizedPoints[i].x;
        point_frame.p[1] = discretizedPoints[i].y;
        point_frame.p[2] = discretizedPoints[i].z;

        int ik_result = ik_solver.CartToJnt(q_init,point_frame,solution);
        if (ik_result)
        {
            std::cout<<"error:无逆解"<<std::endl;
            count ++;
        }
        joint_state.position = {0,0,solution(0),solution(1),solution(2),0,solution(3)};



        joint_state.header.stamp = ros::Time::now(); // 此行代码不能省略，否则rviz上无法运动
        pose.header.stamp = ros::Time::now();

        //处理路径信息
        pose.header.frame_id = "base_link"; // 设置参考坐标系
        pose.pose.position.x = discretizedPoints[i].x;
        pose.pose.position.y = discretizedPoints[i].y;
        pose.pose.position.z = discretizedPoints[i].z;
        pose.pose.orientation.w = 1.0;
        points.push_back(pose);

        publishPath(nh, "base_link", points);
        pub_points_in_line.publish(joint_state);
        

        ros::spinOnce();
        rate.sleep();

    }
    std::cout<<"无逆解的个数："<<count<<std::endl;

    return 0;
}
#pragma endregion



/*
int main()
{
    Point3D start = {1.0, 2.0, 3.0};
    Point3D end = {4.0, 5.0, 6.0};
    int numPoints = 1000;

    std::vector<Point3D> discretizedPoints = discretizeLine(start, end, numPoints);

    for (const Point3D &point : discretizedPoints)
    {
        std::cout << "x: " << point.x << " y: " << point.y << " z: " << point.z << std::endl;
    }

    return 0;
}
*/
