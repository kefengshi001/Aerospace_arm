// Copyright 2021, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>

#include <rocos_app/drive.h>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <string>
#include <gflags/gflags.h>
#include <kdl/chainiksolverpos_lma.hpp>

using namespace KDL;
using namespace rocos;

DEFINE_string(urdf, "config/modified-urdf.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "Link7", "Tip link name");

boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(_joint_num); // 仿真
// boost::shared_ptr< HardwareInterface > hw = boost::make_shared< Hardware >( );  //真实机械臂
rocos::Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

bool isRuning = true;

#pragma region //*测试9  完整上电保护程序
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

namespace rocos
{
    /**
     * @brief 字符串切割函数
     *
     * @param str 待切割字符串
     * @param tokens 结果存储
     * @param delim 切割符
     */

#pragma region // 用来发布节点消息
    void Robot::test()
    {

        ros::NodeHandle nh;
        ros::Publisher publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

        ros::Rate loop_rate(100); // 1 Hz publishing rate

        while (isRuning)
        {
            if (signal(SIGINT, signalHandler) == SIG_ERR)
            {
                std::cout << "\033[1;31m"
                          << "Can not catch SIGINT"
                          << "\033[0m" << std::endl;
            }

            sensor_msgs::JointState joint_state;

            // Set joint names
            joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

            // Set joint positions
            joint_state.position = {getJointPosition(0), getJointPosition(1), getJointPosition(2),
                                    getJointPosition(3), getJointPosition(4), getJointPosition(5), getJointPosition(6)};

            // Set header timestamp
            joint_state.header.stamp = ros::Time::now();

            publisher.publish(joint_state);

            loop_rate.sleep();
        }

        // ros::shutdown();
    }
#pragma endregion

#pragma region //*将输出角度的1、2、6关节锁死*//
    JntArray q_standard2q_target(JntArray q_standard)
    {
        KDL::JntArray q_target(_joint_num);
        //** 1、2、6关节锁死 **//
        q_target(0) = 0 * M_PI / 180;
        q_target(1) = 0 * M_PI / 180;
        q_target(2) = q_standard(2);
        q_target(3) = q_standard(3);
        q_target(4) = q_standard(4);
        q_target(5) = 0 * M_PI / 180;
        q_target(6) = q_standard(6);

        return q_target;
    }
#pragma endregion

#pragma region // move_j实现     KDL::JntArray q
    void move_j(JntArray q, double speed = 0.8, double acceleration = 1.4)
    {
        JntArray q_target = q_standard2q_target(q);
        robot.MoveJ(q_target, speed, acceleration, 0, 0, false);
    }
#pragma endregion

#pragma region // 运动学求解器，限制住1、2、6关节求逆解
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;

    KDL::ChainIkSolverPos_LMA Create_IK_Solver()
    {

        std::string fixed_urdf_file = "src/my_ros_package/config/modified-urdf-fixed.urdf";
        if (!kdl_parser::treeFromFile(fixed_urdf_file, kdl_tree)) // 建立tree
        {
            // 加载URDF文件失败，处理错误
            std::cout << "加载URDF文件失败" << std::endl;
            exit(0);
        }

        // 提取机器人链信息
        std::string base_link = "base_link"; // 设置基准链接
        std::string end_link = "Link7";      // 设置终端链接

        if (!kdl_tree.getChain(base_link, end_link, kdl_chain)) // 建立运动链
        {
            // 获取机器人链失败，处理错误
            std::cout << "获取机器人链失败" << std::endl;
            exit(0);
        }

        // 创建逆向运动学求解器
        KDL::ChainIkSolverPos_LMA ik_solver(kdl_chain);
        return ik_solver;
    }

    KDL::ChainFkSolverPos_recursive Creat_FK_solver()
    {

        std::string fixed_urdf_file = "src/my_ros_package/config/modified-urdf-fixed.urdf";
        if (!kdl_parser::treeFromFile(fixed_urdf_file, kdl_tree)) // 建立tree
        {
            // 加载URDF文件失败，处理错误
            std::cout << "加载URDF文件失败" << std::endl;
            exit(0);
        }

        // 提取机器人链信息
        std::string base_link = "base_link"; // 设置基准链接
        std::string end_link = "Link7";      // 设置终端链接

        if (!kdl_tree.getChain(base_link, end_link, kdl_chain)) // 建立运动链
        {
            // 获取机器人链失败，处理错误
            std::cout << "获取机器人链失败" << std::endl;
            exit(0);
        }

        // 创建逆向运动学求解器
        KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
        return fk_solver;
    }

#pragma endregion

#pragma region // 离散空间中两点    pose_y_left.p:[   -0.644451,     3.55711, 1.84511e-05]      pose_y_right.p:[   -0.644451,     8.87911, 1.84511e-05]

    struct Point3D // 创建一个存放位姿的结构体，后期方便离散和move_l的输入
    {
        const double x = -0.644451;
        double y, z, Roll, Pitch, Yaw;
        // const double Pitch = 0;

        Point3D() {} // 无参数构造函数

        Point3D(double yVal, double zVal, double rollVal, double pitchVal, double YawVal)
            : y(yVal), z(zVal), Roll(rollVal), Pitch(pitchVal), Yaw(YawVal)
        {
            // 在构造函数的初始化列表中完成非常量成员的赋值
        }
    };

    double dt = 0.0001; // 离散线段的步长

    // 将输入的Point3D的对象转化成KDL::Frame 的形式<辅助中间类函数>,用Frame来求逆解。
    KDL::Frame Point3D2Frame(Point3D &pose)
    {
        KDL::Frame target_frame_pose;

        target_frame_pose.p[0] = pose.x;
        target_frame_pose.p[1] = pose.y;
        target_frame_pose.p[2] = pose.z;
        target_frame_pose.M = KDL::Rotation::RPY(pose.Roll, pose.Pitch, pose.Yaw);
        return target_frame_pose;
    }

    // 将当前末端法兰转化成Point3D的对象作为离散线段的起点
    Point3D Flange_Frame2Point3D()
    {
        KDL::Frame Flange_pose = robot.getFlange(); // 获取末端法兰盘位姿

        //***********************打印法兰盘位姿对应关节角***************************************//

        KDL::ChainIkSolverPos_LMA ik_solver = Create_IK_Solver(); // 创建逆运动学求解器
        KDL::JntArray q_init(kdl_chain.getNrOfJoints());          // 注意kdl_chain.getNrOfJoints()放在创建逆运动学求解器后面，Create_IK_Solver()会给kdl_chain赋值，放前面则为空。
        KDL::JntArray solution(kdl_chain.getNrOfJoints());

        ik_solver.CartToJnt(q_init, Flange_pose, solution);

        //*********测试法兰盘**************//
        // std::cout<<"Flange_pose对应的关节角为："<<std::endl;
        // for (int i = 0; i < 4; i++)
        // {
        //     std::cout << solution(i) << "\t";
        // }
        // std::cout << std::endl;

        //**************将当前末端法兰盘位姿转化成Point3D的结构当做起始点用于去离散直线******************//
        Point3D start;
        // start.x = Flange_pose.p[0];
        start.y = Flange_pose.p[1];
        start.z = Flange_pose.p[2];
        // double void_num; // 由于Pitch是const类型，不能修改其值，因此创建一个变量用于临时存放.GetRPY的值，void_num值无作用
        Flange_pose.M.GetRPY(start.Roll, start.Pitch, start.Yaw);
        // std::cout<<"start.Roll ="<<start.Roll<<"\t"<<"start.Pitch ="<<"\t"<<start.Pitch<<"\t"<<"start.Yaw = "<<start.Yaw<<"\n";
        // std::cout<<"start.Roll ="<<start.Roll<<"\t"<<"void_num = "<<void_num<<"\t"<<"start.Yaw = "<<start.Yaw<<"\n";

        return start;
    }

#pragma region //// RPY转化成 Eigen::Quaterniond类型的四元素,便于后续插值
    Eigen::Quaterniond RPY2Quat(double roll, double pitch, double yaw)
    {
        // 将RPY转化成旋转矩阵
        KDL::Rotation rotation = KDL::Rotation::RPY(roll, pitch, yaw);

        // 将旋转矩阵转化成四元素<double类型>
        double qx;
        double qy;
        double qz;
        double qw;
        rotation.GetQuaternion(qx, qy, qz, qw);
        Eigen::Quaterniond V;
        V.x() = qx;
        V.y() = qy;
        V.z() = qz;
        V.w() = qw;
        return V;
    }
#pragma endregion

#pragma region ////将 Eigen::Quaterniond类型的四元素转化成旋转矩阵
    KDL::Rotation Quaterniond2Rotation(Eigen::Quaterniond &quaternion)
    {
        KDL::Rotation quat2rot;
        double qx;
        double qy;
        double qz;
        double qw;

        qx = quaternion.x();
        qy = quaternion.y();
        qz = quaternion.z();
        qw = quaternion.w();
        return quat2rot = KDL::Rotation::Quaternion(qx, qy, qz, qw);
    }
#pragma endregion

    // -------------------------------------------------------------------------------
    // 离散空间中两点构成的直线并将离散点转化成frame再逆运动学求解到JntArray存放在vector中
    std::vector<KDL::JntArray> discretizeLine(const Point3D &start, const Point3D &end, double dt)
    {

        KDL::ChainIkSolverPos_LMA ik_solver = Create_IK_Solver(); // 创建逆运动学求解器
        std::vector<KDL::JntArray> discretePoints;                // 创建<KDL::JntArray>类型容器用来存放关节角

        // 1.将起点和终点的RPY值转化成Eigen::Quaterniond类型的四元素
        // Eigen::Quaterniond start_quat(1.0, 0.0, 0.0, 0.0); // 假设单位四元数
        // Eigen::Quaterniond end_quat(0.0, 1.0, 0.0, 0.0);   // 假设单位四元数
        Eigen::Quaterniond start_quat = RPY2Quat(start.Roll, start.Pitch, start.Yaw);
        Eigen::Quaterniond end_quat = RPY2Quat(end.Roll, end.Pitch, end.Yaw);

        // 计算离散点个数
        double distance = sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2) + pow(end.z - start.z, 2));
        long long int numSteps = static_cast<int>(distance / dt);
        std::cout << "numSteps = " << numSteps << std::endl;
        // sleep(2);

        // 计算每一项的步长
        //  double stepX = (end.x - start.x) / numSteps;
        double stepY = (end.y - start.y) / numSteps;
        double stepZ = (end.z - start.z) / numSteps;
        // 四元素的增量放置到循环里面

        KDL::JntArray q_init(kdl_chain.getNrOfJoints()); // 四个元素
        KDL::JntArray solution(kdl_chain.getNrOfJoints());
        KDL::JntArray discrete_jointarray(_joint_num); // 七个元素

        // 初始化q_init
        q_init(0) = robot.getJointPosition(2);
        q_init(1) = robot.getJointPosition(3);
        q_init(2) = robot.getJointPosition(4);
        q_init(3) = robot.getJointPosition(6);

        // long int count = 1;

        // //************************************************************************//
        // std::ofstream csv_record("vel.csv");

        // if (!csv_record.is_open())
        // {
        //     std::cout << "打开失败" << std::endl;
        //     // flag_loop = false;
        // }

        // //**********************************************************************//

        for (int i = 0; i <= numSteps; ++i)
        {
            Point3D point;
            double t = static_cast<double>(i) / numSteps; // 四元素增量
            // 2.1 插值
            //  point.x = start.x + stepX * i;
            point.y = start.y + stepY * i;
            point.z = start.z + stepZ * i;
            Eigen::Quaterniond interpolated_orientation = start_quat.slerp(t, end_quat);
            // 2.2 将四元素转化成KDL::Rotation 类型的旋转矩阵
            KDL::Frame M_Frame;
            M_Frame.M = Quaterniond2Rotation(interpolated_orientation);
            // 2.3 将旋转矩阵和向量p合并成转换矩阵
            M_Frame.p =KDL::Vector(point.x, point.y, point.z);

            // KDL::Frame mild_frame = Point3D2Frame(point); // 将Point3D转化为KDL::Frame
            // KDL::JntArray mild_q;

            // std::cout<<"robot.getFlange().p = "<<robot.getFlange().p<<std::endl;
            // std::cout<<"robot.getFlange().M = "<<robot.getFlange().M<<std::endl;

            // std::cout<<"mild_frame.p = "<<mild_frame.p<<std::endl;
            // std::cout<<"mild_frame.M = "<<mild_frame.M<<std::endl;

            // if (count == 1) // 将当前位置关节角给q_init
            // {
            //     q_init(0) = robot.getJointPosition(2);
            //     q_init(1) = robot.getJointPosition(3);
            //     q_init(2) = robot.getJointPosition(4);
            //     q_init(3) = robot.getJointPosition(6);
            // }

            // std::cout << "q_init(i)"<<"\t";
            // for (int i = 0; i < 4; i++)
            // {
            //     std::cout << q_init(i) << "\t";
            // }
            // std::cout << std::endl;

            ////************************************************************************************////
            // 2.4 转换矩阵逆解成关节角
            ik_solver.CartToJnt(q_init, M_Frame, solution);
            ////************************************************************************************////

            // std::cout << "solution(i)"<<"\t";
            // for (int i = 0; i < 4; i++)
            // {
            //     std::cout << solution(i) << "\t";
            // }
            // std::cout << std::endl;

            // sleep(10);
            // 将solution中的四个关节角给  discrete_jointarray，后者直接用于move_j和servoj
            discrete_jointarray(0) = 0;
            discrete_jointarray(1) = 0;
            discrete_jointarray(2) = solution(0);
            discrete_jointarray(3) = solution(1);
            discrete_jointarray(4) = solution(2);
            discrete_jointarray(5) = 0;
            discrete_jointarray(6) = solution(3);
            // //*****************************************************************//
            // csv_record << discrete_jointarray(2) << "\t" << discrete_jointarray(3) << "\t" << discrete_jointarray(4) << "\t" << discrete_jointarray(6) << "\t"
            //            << "\n"
            //            << std::flush;
            // //*****************************************************************//

            robot.servoJ(discrete_jointarray);
            discretePoints.push_back(discrete_jointarray);
            q_init = solution;
    
        }

        // //*****************************************************************//
        // csv_record.close();
        // //*****************************************************************//

        return discretePoints;
    }
    // 需要将frame转化成joint存放，后面走servoj
    //  -----------------------------------------------------------------------
#pragma endregion

#pragma region // move_l实现    std::vector<Point3D> target_point;  <x y z roll pitch yaw>

    void move_l(Point3D target_point) //
    {
        // 获取当前末端位姿作为起点
        Point3D start = Flange_Frame2Point3D();

        std::vector<KDL::JntArray> discretePoints = discretizeLine(start, target_point, dt);

        // for (const auto &point : discretePoints)
        // {
        //     KDL::JntArray q_standard = q_standard2q_target(point);
        //     // for (int i = 0;i < 7; i++)
        //     // {
        //     //     std::cout << q_standard(i) << "\t";
        //     // }
        //     std::cout << std::endl;
        //     robot.MoveJ(q_standard,0.8);

        //     // std::cout << "Element: " << point << std::endl;
        // }
    }

#pragma endregion

#pragma region // 显示当前位置关节角和坐标以及对应的RPY值
    void show_joint_and_pose()
    {
        std::cout << "**********关节角及位置信息**********" << std::endl;
        double Roll, Pitch, Yaw;
        KDL::JntArray current_joint(7);
        current_joint(0) = robot.getJointPosition(0);
        current_joint(1) = robot.getJointPosition(1);
        current_joint(2) = robot.getJointPosition(2);
        current_joint(3) = robot.getJointPosition(3);
        current_joint(4) = robot.getJointPosition(4);
        current_joint(5) = robot.getJointPosition(5);
        current_joint(6) = robot.getJointPosition(6);

        std::cout << "current_joint: ";
        for (size_t i = 0; i < current_joint.rows(); ++i)
        {
            std::cout << current_joint(i) << "\t";
        }
        std::cout << std::endl;

        robot.getFlange().M.GetRPY(Roll, Pitch, Yaw);
        std::cout << "getFlange().p: " << robot.getFlange().p << std::endl;
        std::cout << "Roll, Pitch, Yaw : " << Roll << "\t" << Pitch << "\t" << Yaw << std::endl;
        std::cout << "************************************" << std::endl;
    }

#pragma endregion

#pragma region // 运动代码实现
    void run()
    {
        KDL::JntArray q1(_joint_num);
        q1(0) = 0 * deg2rad;
        q1(1) = 0 * deg2rad;
        q1(2) = 30 * deg2rad;
        q1(3) = 90 * deg2rad;
        q1(4) = -45 * deg2rad;
        q1(5) = 0 * deg2rad;
        q1(6) = 45 * deg2rad;
        move_j(q1);
        show_joint_and_pose();

        Point3D target(7.2, 1, -3.1415926, 0, 0);
        move_l(target);
        show_joint_and_pose();

        Point3D target_1(7.2, 0.2, -3.1415926, 0, 0);
        move_l(target_1);
        show_joint_and_pose();

        Point3D target_2(7.2, 0, -3.1415926, 0, -1);
        move_l(target_2);
        show_joint_and_pose();

#pragma region // 读取文本数据并走servoj
               // std::ifstream file("vel.csv");
               // KDL::JntArray q_list(_joint_num);

        // // 检查文件是否成功打开
        // if (!file.is_open())
        // {
        //     std::cerr << "无法打开文件" << std::endl;
        //     exit(0);
        // }

        // std::string line;

        // // 循环遍历每一行数据
        // while (std::getline(file, line))
        // {
        //     std::istringstream iss(line);
        //     std::string field;
        //     std::vector<double> row;

        //     // 使用逗号作为分隔符，将每行数据分割成字段
        //     while (std::getline(iss, field, ','))
        //     {
        //         double value = std::stod(field);
        //         row.push_back(value);
        //     }

        //     // 在这里可以处理每行数据，row 是一个包含字段的字符串向量
        //     for (const auto &value : row)
        //     {
        //         q_list(2) = value;
        //         robot.servoJ(q_list);
        //     }

        //     std::cout << std::endl;
        // }

        // file.close();
#pragma endregion
    }
#pragma endregion

} // namespace rocos
#pragma endregion

/// \brief 处理终端的Ctrl-C信号
/// \param signo

int main(int argc, char *argv[])
{
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //**-------------------------------**//

    auto robotService = RobotServiceImpl::getInstance(&robot);

    ros::init(argc, argv, "joint_state_publisher");

    //------------------------wait----------------------------------
    std::thread thread_test{&rocos::Robot::test, &robot}; // 将一个类的成员函数作为线程函数的入口，
                                                          // 通常需要传递类的实例化对象作为参数，
                                                          // 以便线程函数能够调用该成员函数

    //**** std::thread myThread(&rocos::move_j, std::ref(robot));   ****//往函数里面传参
    std::thread Thread_run(&rocos::run);

    //------------------------wait----------------------------------
    // robotService->runServer();

    thread_test.join();
    Thread_run.join();

    return 0;
}
