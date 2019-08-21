#include "ros/ros.h"

#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <fstream>
#include <string.h>
#include <cmath>

#define PI 3.141592


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
    int current_wp_index;

public:
    Pure_pursuit(const ros::NodeHandle h)
    :nh_c(h), loop_rate(60), current_wp_index(0)
    {
        //set waypoint scv file path from ros parameter
        nh_c.getParam("/pure_pursuit/waypoints/filepath", path_temp);
        filepath = new char[ path_temp.length()+1 ];
        strcpy(filepath, path_temp.c_str());

        //set data from ros param
        nh_c.getParam("/pure_pursuit/driving/look_ahead", lookahead);

        //subscriber for odometry data from particle filter
        sub_pf_odom = nh_c.subscribe("/pf/pose/odom", 10, &Pure_pursuit::subCallback_odom, this);

        get_waypoint();

    }
    ~Pure_pursuit()
    {
        delete []filepath;
        delete []waypoints;
    }
    void get_waypoint();
    void count_waypoint();
    void subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub);
    void find_nearest_wp();
    
};

void Pure_pursuit::get_waypoint()
{
    count_waypoint();
    waypoints = new Point[num_points+1];

    fs.open(filepath, std::ios::in);
    ROS_INFO("openfile\n");

    int i=0;

    while(!fs.eof())
    {
        ROS_INFO("test1");
        
        getline(fs, str_buf, ',');
        waypoints[i].x = std::strtof(str_buf.c_str(),0);
        getline(fs, str_buf, ',');
        waypoints[i].y = std::strtof(str_buf.c_str(),0);
        getline(fs, str_buf, ',');
        waypoints[i].theta = std::strtof(str_buf.c_str(),0);
        getline(fs, str_buf);
        ROS_INFO("get %dth data", i+1);

        ROS_INFO("test2");
        i++;
    }
    ROS_INFO("test3");
    fs.close();

    for(i=0; i<num_points; i++)
    {
        ROS_INFO("%dth line x:%f y:%f theta:%f", i+1, waypoints[i].x, waypoints[i].y, waypoints[i].theta);
    }


}

void Pure_pursuit::count_waypoint()
{
    num_points = 0;
    fs.open(filepath, std::ios::in);
    ROS_INFO("openfile\n");
    while(!fs.eof())
    {
        getline(fs, str_buf, ',');
        getline(fs, str_buf, ',');
        getline(fs, str_buf, ',');
        getline(fs, str_buf);

        num_points++;
    }
    fs.close();
}

void Pure_pursuit::subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub)
{
    double qx, qy, qz, qw;
    double siny_cosp, cosy_cosp;

    // transform quaternion to euler angle -> only for yaw(z-axis)
    qx = msg_sub -> pose.pose.orientation.x;
    qy = msg_sub -> pose.pose.orientation.y;
    qz = msg_sub -> pose.pose.orientation.z;
    qw = msg_sub -> pose.pose.orientation.w;

    siny_cosp = 2.0 * ( qw*qz + qx*qy );
    cosy_cosp = 1.0 - 2.0 * ( qy*qy + qz*qz);
    //z-axis angle
    current_position.theta = atan2(siny_cosp, cosy_cosp);

    current_position.x = msg_sub -> pose.pose.position.x;
    current_position.y = msg_sub -> pose.pose.position.y;


}

void Pure_pursuit::find_nearest_wp()
{

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;

    Pure_pursuit obj(nh);

    

    return 0;

}