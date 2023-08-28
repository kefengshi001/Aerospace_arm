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
#include <kdl/rotational_interpolation_sa.hpp>

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

#pragma region // 逆运动学求解器，限制住1、2、6关节求逆解
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

#pragma endregion

#pragma reigon
    // 插值法
    double dt = 0.001;
    std::vector<KDL::JntArray> generateServojCommands(const Frame &start_pose, const Frame &end_pose)
    {
        KDL::ChainIkSolverPos_LMA ik_solver = Create_IK_Solver();
        KDL::JntArray q_init(kdl_chain.getNrOfJoints());
        KDL::JntArray solution(kdl_chain.getNrOfJoints());
        KDL::JntArray target_solution(_joint_num);

        // 获得机器人当前关节角并初始化q_init为当前机器人关节角
        q_init(0) = robot.getJointPosition(2);
        q_init(1) = robot.getJointPosition(3);
        q_init(2) = robot.getJointPosition(4);
        q_init(3) = robot.getJointPosition(6);

        // 计算离散点个数
        double distance = sqrt(pow(end_pose.x - start_pose.x, 2) + pow(end_pose.y - start_pose.y, 2) + pow(end_pose.z - start_pose.z, 2));
        long long int numSteps = static_cast<int>(distance / dt);

        // Interpolate between start and end poses
        std::vector<KDL::JntArray> interpolated_solutions;

        // Vector start_pos = start_pose.p;
        // Vector end_pos = end_pose.p;
        // Rotation start_rot = start_pose.M;
        // Rotation end_rot = end_pose.M;

        Quaternion start_quat = start_pose.M.GetQuaternion();
        Quaternion end_quat = end_pose.M.GetQuaternion();
        Quaternion interpolated_quat;

        Frame interpolated_pose;

        for (int i = 0; i <= numSteps; ++i)
        {
            double t = static_cast<double>(i) / numSteps;
            interpolated_pose.p = start_pose.p * (1.0 - t) + end_pose.p * t;
            // interpolated_pose.M = Rotation::Interpolate(start_pose.M, end_pose.M, t);

            ik_solver.CartToJnt(q_init, interpolated_pose, solution);
            target_solution = q_standard2q_target(solution);

            robot.servoJ(target_solution);
            interpolated_solutions.push_back(target_solution);
            q_init = solution;
        }
        return interpolated_solutions;
    }
#pragma endregion

#pragma region // move_l实现    std::vector<Point3D> target_point;  <x y z roll pitch yaw>

    void move_l(const Frame &target_pose) //
    {
        // 获取当前末端位姿作为起点
        KDL::Frame start_pose = robot.getFlange();

        // std::vector<KDL::JntArray> discretePoints = discretizeLine(start, target_point, dt);
        generateServojCommands(start_pose, target_pose);
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

#pragma region // 运动代码实现
    void run()
    {
        KDL::JntArray q2(_joint_num);
        KDL::JntArray q1(_joint_num);
        q1(0) = 0 * deg2rad;
        q1(1) = 0 * deg2rad;
        q1(2) = 60 * deg2rad;
        q1(3) = 15 * deg2rad;
        q1(4) = 30 * deg2rad;
        q1(5) = 0 * deg2rad;
        q1(6) = 0 * deg2rad;

        move_j(q2);

        move_j(q1);

        KDL::Frame start_pose = robot.getFlange();
        KDL::Frame end_pose = start_pose;
        end_pose.p.y(7.0);
        end_pose.p.z(0.0);

        move_l(end_pose);

        // std::cout<<robot.getJointPosition(2)<<"\t"<<robot.getJointPosition(3)<<"\t"<<robot.getJointPosition(4)<<"\t"<<robot.getJointPosition(6)<<"\n";

        // Point3D target_pose(7, 0, -3.1415926, 0);
        // // Point3D target_pose;
        // std::cout << target_pose.x << "\t" << target_pose.y << "\t" << target_pose.z << "\t" << target_pose.Roll << "\t" << target_pose.Pitch << "\t" << target_pose.Yaw << "\t" << std::endl;

        //    move_l(target_pose);

        // std::ifstream file("vel[2].csv");

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
        //         q1(2) = value;
        //         robot.servoJ(q1);
        //     }

        //     std::cout << std::endl;
        // }

        // file.close();
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
