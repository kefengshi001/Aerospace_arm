#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

class ArmControllerSwitchNode
{
public:
    ArmControllerSwitchNode() : current_controller_("base_link"), rate_(30)
    {
        ros::NodeHandle nh("~");

        // Subscribe to controller switch commands and joint state messages
        switch_sub_ = nh.subscribe("controller_switch", 1, &ArmControllerSwitchNode::switchCallback, this);
        joint_states_sub_ = nh.subscribe("joint_states", 1, &ArmControllerSwitchNode::jointStatesCallback, this);

        // Advertise a joint trajectory topic to publish smooth transitions
        joint_trajectory_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1);

        // Initialize joint angles
        for (int i = 0; i < num_joints_; ++i)
        {
            joint_angles_[i] = 0.0;
        }

        // Start the main control loop
        controlLoop();
    }

    void switchCallback(const std_msgs::String::ConstPtr& msg)
    {
        if (msg->data == "tcp" || msg->data == "base_link")
        {
            switchController(msg->data);
        }
        else
        {
            ROS_WARN("Invalid control switch command: %s", msg->data.c_str());
        }
    }

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        if (msg->name.size() != num_joints_ || msg->position.size() != num_joints_)
        {
            ROS_WARN_ONCE("Joint state message size mismatch.");
            return;
        }

        for (int i = 0; i < num_joints_; ++i)
        {
            joint_angles_[i] = msg->position[i];
        }
    }

    void switchController(const std::string& target_controller)
    {
        if (target_controller != current_controller_)
        {
            ROS_INFO("Switching controller to: %s", target_controller.c_str());

            // Perform smooth transition using linear interpolation
            const double transition_duration = 3.0;  // in seconds
            const double interpolation_rate = 1.0 / rate_;
            const int num_steps = static_cast<int>(transition_duration * rate_);

            // Interpolate joint angles between current and target configurations
            for (int step = 0; step <= num_steps; ++step)
            {
                double t = static_cast<double>(step) / num_steps;
                trajectory_msgs::JointTrajectoryPoint point;
                point.time_from_start = ros::Duration(t * transition_duration);

                for (int i = 0; i < num_joints_; ++i)
                {
                    point.positions.push_back(joint_angles_[i] * (1.0 - t));  // Linear interpolation
                }

                joint_trajectory_.points.push_back(point);
            }

            // Publish joint trajectory for smooth transition
            joint_trajectory_pub_.publish(joint_trajectory_);

            current_controller_ = target_controller;
        }
    }

    void controlLoop()
    {
        ros::Rate loop_rate(rate_);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::Subscriber switch_sub_;
    ros::Subscriber joint_states_sub_;
    ros::Publisher joint_trajectory_pub_;
    std::string current_controller_;
    int num_joints_ = 6;  // Change this based on your robot's joint configuration
    double joint_angles_[6];  // Store current joint angles
    trajectory_msgs::JointTrajectory joint_trajectory_;  // Stores the interpolated trajectory
    double rate_;  // Control loop rate
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_controller_switch_node");
    ArmControllerSwitchNode controller_switch_node;
    ros::spin();
    return 0;
}
