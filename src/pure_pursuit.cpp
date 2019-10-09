#include "ros/ros.h"
#include "pure_pursuit/pure_pursuit.h"

#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"

#include <iostream>
#include <fstream>
#include <string.h>
#include <sstream>
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
:nh_c(h), loop_rate(100), wp_index_current(0)
{
    ROS_INFO("STARTING NODE");
    //set waypoint scv file path from ros parameter
    nh_c.getParam("/pure_pursuit/waypoints/filepath", path_temp);
    filepath = new char[ path_temp.length()+1 ];
    strcpy(filepath, path_temp.c_str());
    //get waypoints data from csv file
    get_waypoint();
    ROS_INFO("GET WAYPOINTS");

    //set data from ros param
    nh_c.getParam("/pure_pursuit/driving/max_look_ahead", lookahead_max);
    nh_c.getParam("/pure_pursuit/driving/min_look_ahead", lookahead_min);
    nh_c.getParam("/pure_pursuit/driving/max_speed", speed_max);
    nh_c.getParam("/pure_pursuit/driving/min_speed", speed_min);
    nh_c.getParam("/pure_pursuit/tuning/DP_angle_proportion", dp_angle_proportion);
    nh_c.getParam("/pure_pursuit/driving/manual_speed_control/mux_size", MSC_MuxSize);

    //get Manual Speed
    get_manualspeed();

    //subscriber and publisher
    sub_pf_odom = nh_c.subscribe("/pf/pose/odom", 10, &Pure_pursuit::subCallback_odom, this);
    pub_ack = nh_c.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_1", 5);
    pub_dp_mark = nh_c.advertise<visualization_msgs::Marker>("/desired_point/marker", 0);

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
    delete []ManualSpeedArray;
}

void Pure_pursuit::get_waypoint()
{
    count_waypoint();
    waypoints = new Point[ num_points ];

    fs.open(filepath, std::ios::in);
    ROS_INFO("openfile\n");

    int i=0;

    while(!fs.eof())
    {
      
        getline(fs, str_buf, ',');
        waypoints[i].x = std::strtof(str_buf.c_str(),0);
        getline(fs, str_buf, ',');
        waypoints[i].y = std::strtof(str_buf.c_str(),0);
        //getline(fs, str_buf, ',');
        getline(fs, str_buf);
	waypoints[i].theta = std::strtof(str_buf.c_str(),0);        
	ROS_INFO("get %dth data", i+1);

        i++;
    }
    ROS_INFO("test3");
    fs.close();

    for(i=0; i<num_points; i++)
    {
        ROS_INFO("%dth line x:%f y:%f theta:%f", i+1, waypoints[i].x, waypoints[i].y, waypoints[i].theta);
    }
    ROS_INFO("FINISH PRINTING WAYPOINTS");

}

//get manully controlled speed data array from rosparam
void Pure_pursuit::get_manualspeed()
{
    ManualSpeedArray = new ManualSpeed[MSC_MuxSize];
    int i=0;
    std::string str_temp;
    for(i=0; i<MSC_MuxSize; i++)
    {
        std::stringstream ss;
        ss << (i+1);
        str_temp = ss.str();

        std::string region = "/region_";
        region.append(str_temp);
        std::string starting_wp;
        std::string ending_wp;
        std::string max_speed;
        std::string min_speed;

        starting_wp = "/pure_pursuit/driving/manual_speed_control" + region + "/starting_wp";
        ending_wp = "/pure_pursuit/driving/manual_speed_control" + region + "/ending_wp";
        max_speed = "/pure_pursuit/driving/manual_speed_control" + region + "/max_speed";
        min_speed = "/pure_pursuit/driving/manual_speed_control" + region + "/min_speed";

        nh_c.getParam(starting_wp, ManualSpeedArray[i].starting_wp);
        nh_c.getParam(ending_wp, ManualSpeedArray[i].ending_wp);
        nh_c.getParam(max_speed, ManualSpeedArray[i].max_speed);
        nh_c.getParam(min_speed, ManualSpeedArray[i].min_speed);

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
//        getline(fs, str_buf, ',');
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

    
}


/*
Set the wp_index_current to nearest waypoint from the car
*/
void Pure_pursuit::find_nearest_wp()
{
    double temp_distance;
    Point transformed_nearest_point;

    int wp_index_temp = wp_index_current;    

    nearest_distance = getDistance(waypoints[wp_index_temp], current_position);
    
    while(1)
    {
        wp_index_temp++;
     
        if(wp_index_temp >= num_points) wp_index_temp = 0;

        temp_distance = getDistance(waypoints[wp_index_temp], current_position);

        if(temp_distance < nearest_distance)
        {
            nearest_distance = temp_distance;
            wp_index_current = wp_index_temp;
        }
        else if(temp_distance > (nearest_distance + CURRENT_WP_CHECK_OFFSET) || (wp_index_temp == wp_index_current)){ break; } 
    }

    //check the position (right or left) of the path from the car position
    transformed_nearest_point = transformPoint(current_position, waypoints[wp_index_current]);
    if(transformed_nearest_point.x < 0) nearest_distance *= -1;

    ROS_INFO("Nearest wp : %dth wp", wp_index_current+1);
    ROS_INFO("WP Position - x:%f   y:%f", waypoints[wp_index_current].x, waypoints[wp_index_current].y);
    ROS_INFO("CR Position - x:%f   y:%f", current_position.x, current_position.y);
    ROS_INFO("distacne : %f", nearest_distance);
}


void Pure_pursuit::get_dx()
{
    Point wp_min, wp_max;
    wp_min = find_lookahead_wp(lookahead_min);
    wp_max = find_lookahead_wp(lookahead_max);

    wp_min = transformPoint(current_position, wp_min);
    wp_max = transformPoint(current_position, wp_max);

    dx = wp_max.x - wp_min.x;
}

void Pure_pursuit::get_lookahead_desired()
{
    lookahead_desired = exp(-(DX_GAIN*fabs(dx)-log(lookahead_max - lookahead_min))) + lookahead_min;
}


Point Pure_pursuit::find_lookahead_wp(double length)
{
    int wp_index_temp;
    double distance;
    wp_index_temp = wp_index_current;
    while(1)
    {
        if(wp_index_temp >= num_points) wp_index_temp = 0;

        distance = getDistance(waypoints[wp_index_temp], current_position);
        
        if(distance >= length) break;

        wp_index_temp++;
    }
    return waypoints[wp_index_temp];
}


void Pure_pursuit::find_desired_wp()
{
    int wp_index_temp;
    double distance;
    wp_index_temp = wp_index_current;
    while(1)
    {
        if(wp_index_temp >= num_points) wp_index_temp = 0;

        distance = getDistance(waypoints[wp_index_temp], current_position);
        if(distance >= lookahead_desired)
        {
            if(wp_index_temp-2 >= 0 && wp_index_temp+2 < num_points)
            {
                waypoints[wp_index_temp].theta = atan((waypoints[wp_index_temp+2].y-waypoints[wp_index_temp-2].y)/(waypoints[wp_index_temp+2].x-waypoints[wp_index_temp-2].x));
            }
            desired_point = waypoints[wp_index_temp];
            actual_lookahead = distance;
            break;
        }
        wp_index_temp++;
    }
    ROS_INFO("desired point : %dth point", wp_index_temp+1);
 //   ROS_INFO("lookahead - actual : %f,  desired : %f", actual_lookahead, lookahead_desired);
    ROS_INFO("desired x : %f    desired y : %f", desired_point.x, desired_point.y);

}


// 나중에 publisher 를 새로 만들게 되면 그땐 거리 다시 구하는거 추가해야할지 고려
void Pure_pursuit::find_path()
{
    if(transformed_desired_point.x > 0)
    {
        goal_path_radius = pow(actual_lookahead, 2)/(2*transformed_desired_point.x);
        goal_path_theta = asin( transformed_desired_point.y/goal_path_radius );
        steering_direction = -1;
        ROS_INFO("right cornering");
    }
    else
    {
        goal_path_radius = pow(actual_lookahead, 2)/(-2*transformed_desired_point.x);
        goal_path_theta = asin( transformed_desired_point.y/goal_path_radius );
        steering_direction = 1;
        ROS_INFO("left cornering");
    }
    ROS_INFO("path radius : %f", goal_path_radius);
}

void Pure_pursuit::driving()
{
    ROS_INFO("start driving");
    while(ros::ok())
    {
        ros::spinOnce();
        pub_driving_msg.header.stamp = ros::Time::now();
        find_nearest_wp();
        get_dx();
        ROS_INFO("dx value = %f,  abs(dx) %f", dx, fabs(dx));
        get_lookahead_desired();
        find_desired_wp();

        publishDPmarker();

        transformed_desired_point = transformPoint(current_position, desired_point);

        ROS_INFO("tfpoint - x : %f      y : %f", transformed_desired_point.x, transformed_desired_point.y);
    
        find_path();
        setSteeringAngle();
        setSpeed();
        pub_ack.publish(pub_driving_msg);

        ROS_INFO("send speed %f, servo %f", pub_driving_msg.drive.speed, pub_driving_msg.drive.steering_angle);
        

        std::cout<<std::endl;

        loop_rate.sleep();
    }
}



void Pure_pursuit::setSteeringAngle()
{
    tuneSteeringAngle();
    steering_angle = atan( RACECAR_LENGTH / goal_path_radius );
//    ROS_INFO("steering angle : %f", steering_angle);
    pub_driving_msg.drive.steering_angle = (double)steering_direction * steering_angle;

}

void Pure_pursuit::tuneSteeringAngle()
{
    double steering_slope;
    double controlled_slope;

    if(( nearest_distance > 0 && transformed_desired_point.theta < (PI/2) ) || ( nearest_distance < 0 && transformed_desired_point.theta > (PI/2) ))
    {/*codes when the right cornering with right path, left cornering with left path*/
        ROS_INFO("angle tunning X");
    }
    else
    {
//        steering_slope = PI/2 - pub_driving_msg.drive.steering_angle;
        steering_slope = PI/2 - 2 * asin(actual_lookahead/(2*goal_path_radius));
        controlled_slope = dp_angle_proportion * transformed_desired_point.theta + (1 - dp_angle_proportion) * steering_slope;
    
        ROS_INFO("original goal path radius : %f", goal_path_radius);
        goal_path_radius = actual_lookahead/(2*sin(PI/4 - controlled_slope/2));

//        pub_driving_msg.drive.steering_angle = PI/2 - controlled_slope;
    }
    ROS_INFO("DP slope : %f ST slope : %f", transformed_desired_point.theta, steering_slope);
    ROS_INFO("controlled slope : %f", controlled_slope);
    
}

void Pure_pursuit::setSpeed()
{
    double controlled_speed_max;
    double controlled_speed_min;
    int i;
    controlled_speed_max = speed_max;
    controlled_speed_min = speed_min;
    for(i = 0; i<MSC_MuxSize; i++)
    {
        if((wp_index_current > ManualSpeedArray[i].starting_wp) && (wp_index_current < ManualSpeedArray[i].ending_wp))
        {
            ROS_INFO("%dth region speed control", i+1);
            controlled_speed_max = ManualSpeedArray[i].max_speed;
            controlled_speed_min = ManualSpeedArray[i].min_speed;
            break;
        }
    }

    pub_driving_msg.drive.speed = exp(-(DX_GAIN*fabs(dx)-log(controlled_speed_max - controlled_speed_min))) + controlled_speed_min;
}

void Pure_pursuit::publishDPmarker()
{
    Dp_Marker.header.frame_id = "map";
    Dp_Marker.header.stamp = ros::Time();
    Dp_Marker.ns = "pure_pursuit";
    Dp_Marker.id = 1;
    Dp_Marker.type = visualization_msgs::Marker::SPHERE;
    Dp_Marker.action = visualization_msgs::Marker::ADD;
    Dp_Marker.pose.position.x = desired_point.x;
    Dp_Marker.pose.position.y = desired_point.y;
    Dp_Marker.pose.position.z = 0.1;
    Dp_Marker.pose.orientation.x = 0.0;
    Dp_Marker.pose.orientation.y = 0.0;
    Dp_Marker.pose.orientation.z = 0.0;
    Dp_Marker.pose.orientation.w = 1.0;
    Dp_Marker.scale.x = 0.1;
    Dp_Marker.scale.y = 0.1;
    Dp_Marker.scale.z = 0.1;
    Dp_Marker.color.a = 1.0; // Don't forget to set the alpha!
    Dp_Marker.color.r = 1.0;
    Dp_Marker.color.g = 0.0;
    Dp_Marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    pub_dp_mark.publish( Dp_Marker );
}


} //namespace pure_pursuit
