#include "ros/ros.h"
#include "d11310205/AddTwoNums.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_two_ints_client");
    if (argc != 4) {
        ROS_INFO("usage: add_two_ints_client X Y");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<d11310205::AddTwoNums>("multiply_two_ints");
    d11310205::AddTwoNums srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);

    if (client.call(srv)) {
        ROS_INFO("Product: %ld", (long int)srv.response.product);
    } else {
        ROS_ERROR("Failed to call service multiply_two_ints");
        return 1;
    }

    return 0;
}

