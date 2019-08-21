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

double getDistance(Point A, Point B)
{
    double dx, dy;
    dx = A.x - B.x;
    dy = A.y - B.y;
    return sqrt( pow(dx,2) + pow(dy,2));
}

Point transformPoint(Point origin_point, Point target_point)
{
    Point tf_point;
    double dx, dy, theta, dtheta;
    
    theta = PI/2 - origin_point.theta;

    dx = target_point.x - origin_point.x;
    dy = target_point.y - origin_point.y;
    dtheta = target_point.theta + theta;

    tf_point.x = dx * cos(theta) - dy * sin(theta);
    tf_point.y = dx * sin(theta) + dy * cos(theta);
    tf_point.theta = dtheta;

    return tf_point;

}


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

    //subscriber and publisher
    sub_pf_odom = nh_c.subscribe("/pf/pose/odom", 10, &Pure_pursuit::subCallback_odom, this);
    pub_ack = nh_c.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_1", 5);

    pub_driving_msg.header.stamp = ros::Time::now();
    pub_driving_msg.header.frame_id = "base_link";
    pub_driving_msg.drive.speed = 0;
    pub_driving_msg.drive.acceleration = 1;
    pub_driving_msg.drive.jerk = 1;
    pub_driving_msg.drive.steering_angle = 0;
    pub_driving_msg.drive.steering_angle_velocity = 1;

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
    find_desired_wp(lookahead);
    transformed_desired_point = transformPoint(current_position, desired_point);

    ROS_INFO("tfpoint - x : %f      y : %f", transformed_desired_point.x, transformed_desired_point.y);
    
    find_path();
    drive_test();
    std::cout<<std::endl;
}


/*
Set the wp_index_current to nearest waypoint from the car
*/
void Pure_pursuit::find_nearest_wp()
{
    double nearest_distance;
    double temp_distance;

    int wp_index_temp = wp_index_current;
    
    nearest_distance = getDistance(waypoints[wp_index_temp], current_position);
    
    while(1)
    {
        wp_index_temp++;
     
        temp_distance = getDistance(waypoints[wp_index_temp], current_position);
        //ROS_INFO("temp distance : %f", temp_distance);

        if(temp_distance < nearest_distance)
        {
            nearest_distance = temp_distance;
            wp_index_current = wp_index_temp;
        }
        else if(temp_distance > nearest_distance + CURRENT_WP_CHECK_OFFSET | wp_index_temp > num_points) break; 
    }

    ROS_INFO("Nearest wp : %dth wp", wp_index_current+1);
    ROS_INFO("WP Position - x:%f   y:%f", waypoints[wp_index_current].x, waypoints[wp_index_current].y);
    ROS_INFO("CR Position - x:%f   y:%f", current_position.x, current_position.y);
    ROS_INFO("distacne : %f", nearest_distance);
}

void Pure_pursuit::find_desired_wp(double length)
{
    int wp_index_temp;
    double distance;
    wp_index_temp = wp_index_current;
    while(1)
    {
        distance = getDistance(waypoints[wp_index_temp], current_position);
        if(distance >= length)
        {
            desired_point = waypoints[wp_index_temp];
            actual_lookahead = distance;
            break;
        }
        wp_index_temp++;
    }
    ROS_INFO("desired point : %dth point", wp_index_temp);
    ROS_INFO("actual_lookahead : %f, lookahead : %f", actual_lookahead, lookahead);
    ROS_INFO("desired x : %f    desired y : %f", desired_point.x, desired_point.y);

}

// 나중에 publisher 를 새로 만들게 되면 그땐 거리 다시 구하는거 추가해야할지 고려
void Pure_pursuit::find_path()
{
    if(transformed_desired_point.x > 0)
    {
        goal_path_radius = pow(actual_lookahead, 2)/(2*transformed_desired_point.x);
        goal_path_theta = asin( transformed_desired_point.y/goal_path_radius );
        steering_direction = 0;
        ROS_INFO("right cornering");
    }
    else
    {
        goal_path_radius = pow(actual_lookahead, 2)/(-2*transformed_desired_point.x);
        goal_path_theta = asin( transformed_desired_point.y/goal_path_radius );
        steering_direction = 1;
        ROS_INFO("left cornering");
    }
    ROS_INFO("path radius : %f     path theta : %f", goal_path_radius, goal_path_theta);
}


void Pure_pursuit::drive_test()
{
    pub_driving_msg.drive.speed = 1.0;
    if(steering_direction == 0)
    {
        pub_driving_msg.drive.steering_angle = -goal_path_theta;
    }
    else
    {
        pub_driving_msg.drive.steering_angle = goal_path_theta;
    }

    pub_ack.publish(pub_driving_msg);
}


} //namespace pure_pursuit