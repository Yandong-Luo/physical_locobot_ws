#include "controller/controller_node.h"

Controller::Controller(ros::NodeHandle* nodeHandle):nh_(*nodeHandle)
{
    // initial subse
    // clock_sub = nh_.subscribe("/clock", 10, &SlamController::sub_clockCallback, this);

    tennis_ball_info = nh_.subscribe("/tennis_ball_distance_calculator/ball_info",10,&Controller::ball_info_Callback,this);
    
    // get param from launch
    nh_.getParam("/slam_controller_node/pan_angle",angle_value);

    ROS_INFO("pan_angle:%f",angle_value);

    send_angle = false;

    paused_state = true;

    FoundBall = false;

    controller_client = nh_.serviceClient<interbotix_controller_service::MoveItPlan>("/locobot/moveit_plan");
}

void Controller::request_set_camera_tilt_angle(float angle)
{
    if (angle != angle_value)
    {
        // update current angle_value
        angle_value = angle;

        srv.request.cmd = interbotix_controller_service::MoveItPlan::Request::CMD_CONTROL_CAMERA_ANGLE;
        std_msgs::Float64 value;
        value.data = angle;
        srv.request.camera_angle = value;

        if (controller_client.call(srv))
        {
            ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
        }
        else
        {
            ROS_ERROR("Failed to call service to change tilt angle\n");
        }
    }
}

void Controller::request_planning_joint_pose()
{
    srv.request.cmd = interbotix_controller_service::MoveItPlan::Request::CMD_PLAN_JOINT_POSE;
    
    // {1.55, 0, -1.1, 0, 0.5, 0}
    srv.request.joint_vector = {0,0.5,0,-1.1,0,1.55};

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to planning joint position\n");
    }
}

void Controller::request_planning_joint_pose_to_sleep()
{
    srv.request.cmd = interbotix_controller_service::MoveItPlan::Request::CMD_PLAN_JOINT_POSE_TO_SLEEP;
    
    // {1.55, 0, -1.1, 0, 0.5, 0}
    // srv.request.joint_vector = {0,0.5,0,-1.1,0,1.55};

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to planning joint position to sleep\n");
    }
}

void Controller::request_planning_joint_pose_to_home()
{
    srv.request.cmd = interbotix_controller_service::MoveItPlan::Request::CMD_PLAN_JOINT_POSE_TO_HOME;
    
    // {1.55, 0, -1.1, 0, 0.5, 0}
    // srv.request.joint_vector = {0,0.5,0,-1.1,0,1.55};

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to planning joint position to home\n");
    }
}

void Controller::request_execute_moveit_plan()
{
    srv.request.cmd = interbotix_controller_service::MoveItPlan::Request::CMD_EXECUTE;

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to execute moveit plan\n");
    }
}

void Controller::request_open_gripper(void)
{
    srv.request.cmd = interbotix_controller_service::MoveItPlan::Request::CMD_OPEN_GRIPPER;

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to open gripper\n");
    }
}

void Controller::request_close_gripper(void)
{
    srv.request.cmd = interbotix_controller_service::MoveItPlan::Request::CMD_CLOSE_GRIPPER;

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to close gripper\n");
    }
}

void Controller::request_pick_up_test(void)
{
    srv.request.cmd = interbotix_controller_service::MoveItPlan::Request::CMD_PICK_UP_TEST;

    if (controller_client.call(srv))
    {
        ROS_INFO("Respone: %s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to do pick up test\n");
    }
}

void Controller::request_pick_up_tennis_ball()
{
    srv.request.cmd = interbotix_controller_service::MoveItPlan::Request::CMD_PICK_UP;
    srv.request.tennis_position = ball_point_info;

    if(controller_client.call(srv))
    {
        ROS_INFO("Respone:%s\n", srv.response.msg.data.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service to pick up tennis ball");
    }
}

void Controller::ball_info_Callback(const tennis_ball_distance_calculator::ball_info::ConstPtr& msg)
{
    const tennis_ball_distance_calculator::ball_info ball_info = *msg;
    
    // didn't found a tennis ball
    if (ball_info.found_tennis == false)
    {
        // FoundBall = false;
        return;
    }
    else{
        FoundBall = true;
        ball_point_info = ball_info.ball_pose;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"controller");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);

    spinner.start();

    Controller m_Controller(&nh);

    m_Controller.request_set_camera_tilt_angle(45.0);

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    // m_Controller.request_planning_joint_pose();
    // m_Controller.request_planning_joint_pose_to_home();

    ros::WallDuration(1.0).sleep();

    // m_Controller.request_execute_moveit_plan();

    ros::WallDuration(1.0).sleep();

    // m_Controller.request_pick_up_test();

    // ros::WallDuration(5.0).sleep();

    if(m_Controller.FoundBall == true)
    {
        m_Controller.request_pick_up_tennis_ball();
    }

    // m_Controller.request_open_gripper();

    ros::WallDuration(5.0).sleep();

    // m_Controller.request_open_gripper();

    ros::waitForShutdown();

    return 0;
}