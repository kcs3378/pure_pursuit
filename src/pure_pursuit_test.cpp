#include "ros/ros.h"
#include "pure_pursuit/pure_pursuit_test.h"

#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <fstream>
#include <string.h>
#include <cmath>

namespace pure_pursuit
{

Pure_pursuit::Pure_pursuit(const ros::NodeHandle h)
:nh_c(h), loop_rate(60), wp_index_current(0)
{
    //set waypoint scv file path from ros parameter
    nh_c.getParam("/pure_pursuit/waypoints/filepath", path_temp);
    filepath = new char[ path_temp.length()+1 ];
    strcpy(filepath, path_temp.c_str());
    //get waypoints data from csv file
    get_waypoint();


    //set data from ros param
    nh_c.getParam("/pure_pursuit/driving/look_ahead", lookahead);

    //subscriber for odometry data from particle filter
    sub_pf_odom = nh_c.subscribe("/pf/pose/odom", 10, &Pure_pursuit::subCallback_odom, this);
}

Pure_pursuit::~Pure_pursuit()
{
    delete []filepath;
    delete []waypoints;
}

void Pure_pursuit::get_waypoint()
{
    count_waypoint();
    waypoints = new Point[num_points+1];

    fs.open(filepath, std::ios::in);
    ROS_INFO("openfile\n");

    int i=0;

    while(!fs.eof())
    {
      
        getline(fs, str_buf, ',');
        waypoints[i].x = std::strtof(str_buf.c_str(),0);
        getline(fs, str_buf, ',');
        waypoints[i].y = std::strtof(str_buf.c_str(),0);
        getline(fs, str_buf, ',');
        waypoints[i].theta = std::strtof(str_buf.c_str(),0);
        getline(fs, str_buf);
        ROS_INFO("get %dth data", i+1);

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


/*
Odom data from particle filter to current position data
*/
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

    find_nearest_wp();
}


/*
Set the wp_index_current to nearest waypoint from the car
*/
void Pure_pursuit::find_nearest_wp()
{
    double nearest_distance;
    double temp_distance;
    double dx, dy;

    int wp_index_temp = wp_index_current;
    
    dx = waypoints[wp_index_temp].x - current_position.x;
    dy = waypoints[wp_index_temp].y - current_position.y;
    
    nearest_distance = sqrt( pow(dx , 2) + pow(dy, 2));
    
    while(1)
    {
        dx = waypoints[wp_index_temp].x - current_position.x;
        dy = waypoints[wp_index_temp].y - current_position.y;
        temp_distance = sqrt( pow(dx, 2) + pow(dy, 2));
        //ROS_INFO("temp distance : %f", temp_distance);

        if(temp_distance < nearest_distance)
        {
            nearest_distance = temp_distance;
            wp_index_current = wp_index_temp;
        }
        else if(temp_distance > nearest_distance + CURRENT_WP_CHECK_OFFSET | wp_index_temp > num_points) break; 

        wp_index_temp++;
    }

    ROS_INFO("Nearest wp : %dth wp", wp_index_current+1);
    ROS_INFO("WP Position - x:%f   y:%f", waypoints[wp_index_current].x, waypoints[wp_index_current].y);
    ROS_INFO("CR Position - x:%f   y:%f", current_position.x, current_position.y);
    ROS_INFO("distacne : %f", nearest_distance);
}

void Pure_pursuit::find_desired_wp(double L)
{
    int wp_index_temp;
    double distance;
}

} //namespace pure_pursuit