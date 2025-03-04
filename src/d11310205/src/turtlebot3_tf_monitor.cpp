#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot3_tf_monitor");
    ros::NodeHandle nh;

    // 創建 TF Listener
    tf::TransformListener listener;
    ros::Rate rate(10.0);  // 查詢頻率為 10Hz

    while (ros::ok()) {
        tf::StampedTransform transform;

        try {
            // 查詢 base_link 到 base_scan 的 TF
            listener.lookupTransform("odom", "base_footprint", ros::Time(0), transform);

            ROS_INFO("TurtleBot3 'odom' position in 'base_footprint' frame: x = %.2f, y = %.2f, z = %.2f",
                     transform.getOrigin().x(),
                     transform.getOrigin().y(),
                     transform.getOrigin().z());
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

