#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>

//RPY转化成 Eigen::Quaterniond类型的四元素,便于后续插值
Eigen::Quaterniond RPY2Quat(double roll, double pitch, double yaw)
{
    //将RPY转化成旋转矩阵
    KDL::Rotation rotation = KDL::Rotation::RPY(roll, pitch, yaw);

    //将旋转矩阵转化成四元素<double类型>
    double qx;
    double qy;
    double qz;
    double qw;
    rotation.GetQuaternion(qx,qy,qz,qw);
    Eigen::Quaterniond V;
    V.x() = qx;
    V.y() = qy;
    V.z() = qz;
    V.w() = qw;
    return V;
}


//将 Eigen::Quaterniond类型的四元素转化成旋转矩阵
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




int main()
{
    // 插值步数
    int num_steps = 10;


    //1.将起点和终点的RPY值转化成Eigen::Quaterniond类型的四元素

    // 定义起始姿态和目标姿态的四元数
    Eigen::Quaterniond start_orientation(1.0, 0.0, 0.0, 0.0); // 假设单位四元数
    Eigen::Quaterniond end_orientation(0.0, 1.0, 0.0, 0.0);   // 假设单位四元数

    

  

    // std::vector<Eigen::Quaterniond> interpolated_orientations;

    //2.在循环中实现四元素的插值，四元素转化成旋转矩阵，将旋转矩阵和向量p合成转换矩阵，转换矩阵逆解成关节角，完成servoJ<或者放到外面>
    for (int i = 0; i <= num_steps; ++i)
    {
        //增量
        double t = static_cast<double>(i) / num_steps;
        //2.1 插值
        Eigen::Quaterniond interpolated_orientation = start_orientation.slerp(t, end_orientation);
        //2.2 将四元素转化成KDL::Rotation 类型的旋转矩阵
        KDL::Frame M_Frame;
        M_Frame.M = Quaterniond2Rotation(interpolated_orientation);

        //2.3 将旋转矩阵和向量p合并成转换矩阵


        //2.4 转换矩阵逆解成关节角


        //2.5 完成servoJ
        
        // interpolated_orientations.push_back(interpolated_orientation);
    }

    // // 输出插值后的姿态
    // for (const auto &orientation : interpolated_orientations)
    // {
    //     std::cout << "x: " << orientation.x() << " y: " << orientation.y()
    //               << " z: " << orientation.z() << " w: " << orientation.w() << std::endl;
    // }

    return 0;
}
