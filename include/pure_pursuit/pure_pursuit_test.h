#ifndef __HEADER_pure_pursuit_test_
#define __HEADER_pure_pursuit_test_

#include <fstream>
#include <string.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"

#define PI 3.141592
#define CURRENT_WP_CHECK_OFFSET 5

namespace pure_pursuit
{

typedef struct _Point{
    double x;
    double y;
    double theta;
} Point;


class Pure_pursuit
{
private:
    ros::NodeHandle nh_c;
    ros::Rate loop_rate;
    ros::Subscriber sub_pf_odom;

    //for csv file
    std::fstream fs;
    std::string str_buf;
    std::string path_temp;
    char * filepath;

    //for waypoints
    int num_points;
    Point * waypoints;

    //for driving setting
    double lookahead;

    //for current state
    Point current_position;
    int wp_index_current;
    int wp_index_desired;

public:
    Pure_pursuit(const ros::NodeHandle h);
    ~Pure_pursuit();
    void get_waypoint();
    void count_waypoint();
    void subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub);
    void find_nearest_wp();
    void find_desired_wp(double L);
};

} // namespace pure_pursuit

#endif