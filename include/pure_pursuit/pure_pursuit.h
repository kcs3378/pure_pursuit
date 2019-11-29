#ifndef __HEADER_pure_pursuit_
#define __HEADER_pure_pursuit_

#include <fstream>
#include <string.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"

#define PI 3.141592
#define CURRENT_WP_CHECK_OFFSET 2
#define DX_GAIN 2.5
#define RACECAR_LENGTH 0.325
#define GRAVITY ACCELERATION 9.81

namespace pure_pursuit
{

typedef struct _Point{
    double x;
    double y;
    double theta;
} Point;

typedef struct _Manual_speed{
    int starting_wp;
    int ending_wp;
    double max_speed;
    double min_speed;
} ManualSpeed;

class Pure_pursuit
{
private:
    ros::NodeHandle nh_c;
    ros::Rate loop_rate;
    ros::Subscriber sub_pf_odom;
    ros::Publisher pub_ack;
    ros::Publisher pub_current_mark;
    ros::Publisher pub_dp_mark;

    //for rviz marker
    visualization_msgs::Marker Currnet_Marker;
    visualization_msgs::Marker Dp_Marker;

    //for csv file
    std::fstream fs;
    std::string str_buf;
    std::string path_temp;
    char * filepath;

    //for waypoints
    int num_points;
    Point * waypoints;

    //for manual speed control
    int MSC_MuxSize;
    ManualSpeed *ManualSpeedArray;

    //for driving
    double mu;			// friction coefficient
    double nearest_distance;
    double lookahead_max;
    double lookahead_min;
    double lookahead_desired;
    double dx;      //dx between lookahead_max point and lookahead_min point
    double actual_lookahead;
    double speed_max;
    double speed_min;
    Point desired_point;
    Point transformed_desired_point;
    double goal_path_theta;
    double goal_path_radius;
    int steering_direction; //-1 : right, 1 : left
    double steering_angle;

    double dp_angle_proportion;

    ackermann_msgs::AckermannDriveStamped pub_driving_msg;

    //for current state
    Point current_position;
    int wp_index_current;

    //time
    ros::Time time_now;
    ros::Time time_pre;
    ros::Duration calc_cycle;


public:
    Pure_pursuit(const ros::NodeHandle h);
    ~Pure_pursuit();
    void get_waypoint();
    void get_manualspeed();
    void count_waypoint();
    void subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub);
    void find_nearest_wp();
    Point find_lookahead_wp(double lenght);
    void get_dx();
    void get_lookahead_desired();
    void find_desired_wp();
    void find_path();
    void driving();
    void setSteeringAngle();
    void tuneSteeringAngle();
    void setSpeed();
    void setSpeed_PossibleMaximumTest();
    void publishDPmarker();
};

double getDistance(Point A, Point B);
Point transformPoint(Point origin_point, Point target_point);


} // namespace pure_pursuit

#endif
