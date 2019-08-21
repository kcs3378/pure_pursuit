#include "ros/ros.h"
#include "pure_pursuit/pure_pursuit_test.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;

    pure_pursuit::Pure_pursuit obj(nh);

    ros::spin();

    return 0;

}