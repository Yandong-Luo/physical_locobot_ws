#ifndef _CONTROLLER_NODE_H_
#define _CONTROLLER_NODE_H_

#include <iostream>
#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Time.h>
// 
#include <interbotix_controller_service/MoveItPlan.h>
#include <interbotix_xs_msgs/JointGroupCommand.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tennis_ball_distance_calculator/ball_info.h>

#include <geometry_msgs/PointStamped.h>

# define M_PI       3.14159265358979323846  /* pi */

class Controller
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber tennis_ball_info;
    
    ros::ServiceClient controller_client;

    interbotix_controller_service::MoveItPlan srv;

    geometry_msgs::PointStamped ball_point_info;

    
    
public:
    Controller(ros::NodeHandle* nodehandle);
    ~Controller();

    bool send_angle;
    float angle_value;

    bool paused_state;

    void request_set_camera_tilt_angle(float value);

    void request_planning_joint_pose(void);

    void request_execute_moveit_plan(void);
    
    void request_planning_joint_pose_to_sleep(void);

    void request_planning_joint_pose_to_home(void);

    void request_open_gripper(void);

    void request_close_gripper(void);

    void request_pick_up_tennis_ball(void);

    void request_pick_up_test(void);

    void ball_info_Callback(const tennis_ball_distance_calculator::ball_info::ConstPtr& msg);

    bool FoundBall;
};
Controller::~Controller()
{}
#endif