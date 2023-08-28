#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

nav_msgs::Path readCSVAndGeneratePath(ros::NodeHandle& nh, const std::string& filename, const std::string& frame_id) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        ROS_ERROR("无法打开文件");
        return nav_msgs::Path();
    }

    nav_msgs::Path path;
    path.header.frame_id = frame_id;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string value;

        double x, y, z;
        for (int i = 0; i < 4; ++i) {
            std::getline(iss, value, '\t');
            if (i == 1) x = std::stod(value);
            if (i == 2) y = std::stod(value);
            if (i == 3) z = std::stod(value);
        }

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = path.header.frame_id;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

    file.close();
    return path;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "csv_to_path");
    ros::NodeHandle nh;

    std::string filename = "/home/k/下载/my_kdl-master/app/data.csv"; // 修改为你的文件路径
    std::string frame_id = "base_link"; // 设置参考坐标系

    nav_msgs::Path path = readCSVAndGeneratePath(nh, filename, frame_id);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path_topic_range", 1);
    ros::Rate rate(10); // 发布频率 1Hz

    std::cout << "启动边界" << std::endl;


    while (ros::ok()) {
        path.header.stamp = ros::Time::now();
        path_pub.publish(path);
        rate.sleep();
    }

    return 0;
}
