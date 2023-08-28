#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "cube_marker_publisher");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("cube_marker", 1);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link"; // 参考坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "cube_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = -1;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0; // 改变长方体的高度位置
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 20;
    marker.scale.z = 10;
    marker.color.r = 0.5;
    marker.color.g = 0.4;
    marker.color.b = 0.2;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(); // 持续时间为永久

    std::cout << "启动立方体（墙面）" << std::endl;
    ros::Rate rate(1);
    while (ros::ok()) {
        marker_pub.publish(marker);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
