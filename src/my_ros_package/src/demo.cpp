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

// DEFINE_string(urdf, "config/robot.urdf", "Urdf file path");
DEFINE_string(urdf, "config/modified-urdf.urdf", "Urdf file path");

DEFINE_string(base, "base_link", "Base link name");
// DEFINE_string(tip, "link_7", "Tip link name");
DEFINE_string(tip, "Link7", "Tip link name");


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
            // joint_state.name = {"joint3", "joint4", "joint5", "joint7"};

            
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

    using namespace rocos;


    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //**-------------------------------**//

    boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(_joint_num); // 仿真
    // boost::shared_ptr< HardwareInterface > hw = boost::make_shared< Hardware >( );  //真实机械臂

    Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

    auto robotService = RobotServiceImpl::getInstance(&robot);

    ros::init(argc, argv, "joint_state_publisher");

    //------------------------wait----------------------------------
    // std::thread thread_test{&rocos::Robot::test, &robot};

    //------------------------wait----------------------------------
    robotService->runServer();

    // thread_test.join();

    return 0;
}
