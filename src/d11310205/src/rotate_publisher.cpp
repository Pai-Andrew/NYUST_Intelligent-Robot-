#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rotate_publisher");
    ros::NodeHandle nh;

    // 創建 Publisher
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 設置頻率為 10Hz
    ros::Rate loop_rate(10);

    // 定義 Twist 消息
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 1.0;  // 線速度：1 m/s
    twist_msg.angular.z = 1.0; // 角速度：1 rad/s

    ROS_INFO("Rotate Publisher Started. Publishing angular velocity...");

    while (ros::ok()) {
        // 發布消息
        cmd_vel_pub.publish(twist_msg);
        ROS_INFO("Publishing: linear.x = %.2f, angular.z = %.2f", twist_msg.linear.x, twist_msg.angular.z);
        loop_rate.sleep();
    }

    return 0;
}

