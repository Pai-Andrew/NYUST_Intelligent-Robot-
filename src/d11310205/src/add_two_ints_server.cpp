#include "ros/ros.h"
#include "d11310205/AddTwoNums.h"

bool multiply(d11310205::AddTwoNums::Request &req,
              d11310205::AddTwoNums::Response &res) {
    res.product = req.a * req.b;
    ROS_INFO("Sending back response: [%ld]", (long int)res.product);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("multiply_two_ints", multiply);
    ROS_INFO("Ready to multiply two ints.");
    ros::spin();
    return 0;
}

