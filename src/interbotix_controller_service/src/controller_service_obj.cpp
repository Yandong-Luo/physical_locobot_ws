#include "interbotix_controller_service/controller_service_obj.h"

SimulationController::SimulationController(ros::NodeHandle *node_handle):node(*node_handle)
{
    // camera publisher
    // if u using the simulation, this topic name should be change to /locobot/tilt_controller/command with data type of std_msgs::Float64
    camera_pub = node.advertise<interbotix_xs_msgs::JointGroupCommand>("/locobot/commands/joint_group", 10);

    gripper_pub = node.advertise<interbotix_xs_msgs::JointSingleCommand>("/locobot/commands/joint_single",10);
    
    // service
    srv_moveit_plan = node.advertiseService("moveit_plan",&SimulationController::Command_Process,this);

    static const std::string ARM_PLANNING_GROUP = "interbotix_arm";

    static const std::string GRIPPER_PLANNING_GROUP = "interbotix_gripper";

    arm_move_group = new moveit::planning_interface::MoveGroupInterface(ARM_PLANNING_GROUP);
    arm_joint_model_group = arm_move_group->getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);

    // gripper_move_group = new moveit::planning_interface::MoveGroupInterface(GRIPPER_PLANNING_GROUP);
    // gripper_joint_model_group = gripper_move_group->getCurrentState()->getJointModelGroup(GRIPPER_PLANNING_GROUP);

    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

    std::cout<<"========================================================================"<<std::endl;

    // arm
    std::vector<std::string> all_arm_link_name = arm_move_group->getJoints();

    for (int i=0;i<all_arm_link_name.size();i++)
    {
        std::cout<< all_arm_link_name.at(i)<<std::endl;
    }

    ROS_INFO("======================simulation_controller END effector link:%s\n",arm_move_group->getEndEffectorLink().c_str());

}

/// @brief Destructor for the simulation controller
SimulationController::~SimulationController()
{
    delete arm_move_group;
    delete arm_joint_model_group;

    delete gripper_move_group;
    delete gripper_joint_model_group;
}

/// @brief Publish the the camera's tilt angle to control 
bool SimulationController::publish_camera_tilt_angle(const std_msgs::Float64 angle_data)
{
    interbotix_xs_msgs::JointGroupCommand tilt_info;
    float angle_value = angle_data.data;
    tilt_info.name = "camera";
    angle_value = angle_value*M_PI/180.0f;
    tilt_info.cmd = {0, angle_value};

    camera_pub.publish(tilt_info);
    return true;

    // std::map<std::string, double> tile_joint_pose;
    // tile_joint_pose["tilt"] = angle_value;

    // arm_move_group->setJointValueTarget(tile_joint_pose);
    // bool success = (arm_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // return success;
}

/// @brief planning the position of joint
bool SimulationController::plan_joint_positions(const std::vector<double> joint_group_positions)
{
  arm_move_group->setJointValueTarget(joint_group_positions);
  bool success = (arm_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  return success;
}

/// @brief Planning the position of the end effector
bool SimulationController::plan_end_effector_pose(const geometry_msgs::Pose pose)
{
    arm_move_group->setPoseTarget(pose);
    bool success = (arm_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    return success;
}

/// @brief Return the current end-effector pose relative to the 'world' frame
geometry_msgs::Pose SimulationController::moveit_get_end_effector_pose(void)
{
  return arm_move_group->getCurrentPose().pose;
}

/// @brief Execute a Moveit plan on the robot arm
bool SimulationController::execute_plan(void)
{
  bool success = (arm_move_group->execute(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  return success;
}

/// @brief planning the position of joint to home state
bool SimulationController::plan_joint_pose_to_home(void)
{
    std::map<std::string, double> home_joint_pose;
    home_joint_pose["elbow"] = 0.0;
    home_joint_pose["forearm_roll"] = 0.0;
    home_joint_pose["shoulder"] = 0.0;
    home_joint_pose["waist"] = 0.0;
    home_joint_pose["wrist_angle"] = 0.0;
    home_joint_pose["wrist_rotate"] = 0.0;

    arm_move_group->setJointValueTarget(home_joint_pose);
    bool success = (arm_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return success;
}

/// @brief planning the position of joint to sleep state
bool SimulationController::plan_joint_pose_to_sleep(void)
{
    std::map<std::string, double> sleep_joint_pose;
    sleep_joint_pose["elbow"] = 1.55;
    sleep_joint_pose["forearm_roll"] = 0.0;
    sleep_joint_pose["shoulder"] = -1.1;
    sleep_joint_pose["waist"] = 0.0;
    sleep_joint_pose["wrist_angle"] = 0.5;
    sleep_joint_pose["wrist_rotate"] = 0.0;

    arm_move_group->setJointValueTarget(sleep_joint_pose);
    bool success = (arm_move_group->plan(saved_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return success;
}

/// @brief open the gripper (PWM MODEL)
bool SimulationController::open_gripper(void)
{
    interbotix_xs_msgs::JointSingleCommand gripper_info;

    gripper_info.name="gripper";

    gripper_info.cmd = 250.0;

    gripper_pub.publish(gripper_info);

    return true;
}

/// @brief close the gripper (PWM MODEL)
bool SimulationController::close_gripper(void)
{
    interbotix_xs_msgs::JointSingleCommand gripper_info;

    gripper_info.name="gripper";

    gripper_info.cmd = -250.0;

    gripper_pub.publish(gripper_info);

    return true;
}

// void SimualtionCOntroller::convert_

void SimulationController::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(1);
  posture.joint_names[0] = "left_finger";
//   posture.joint_names[1] = "right_finger";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.5;
  posture.points[0].time_from_start = ros::Duration(2);
  // END_SUB_TUTORIAL
}

void SimulationController::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(1);
  posture.joint_names[0] = "left_finger";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = -0.5;
  posture.points[0].time_from_start = ros::Duration(2);
  // END_SUB_TUTORIAL
}

/// @brief pick up test
bool SimulationController::pick_up_test(void)
{
    add_tennis_ball_as_object();

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "locobot/arm_base_link";
    tf2::Quaternion orientation;
    // orientation.setRPY(0, 0, 0);
    orientation.setRPY(0, 0.0, 0);
    ROS_INFO("=========================================ORIENTATION:%f\n",orientation.getX());
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.546;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.05;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "locobot/arm_base_link";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // // Setting post-grasp retreat
    // // ++++++++++++++++++++++++++
    // /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "locobot/arm_base_link";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);

    closedGripper(grasps[0].grasp_posture);

    arm_move_group->pick("tennis_ball", grasps);

    return true;
}

/// @brief add tennis ball as object
void SimulationController::add_tennis_ball_as_object_test(void)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].header.frame_id = "locobot/arm_base_link";
    collision_objects[0].id = "tennis_ball";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].SPHERE;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.02;
    collision_objects[0].primitives[0].dimensions[1] = 0.02;
    collision_objects[0].primitives[0].dimensions[2] = 0.02;

    /* Define the pose of the object. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.7;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.04;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[0].operation = collision_objects[2].ADD;

    planning_scene_interface->applyCollisionObjects(collision_objects);
}

/// @brief add tennis ball as object
void SimulationController::add_tennis_ball_as_object()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].header.frame_id = "locobot/arm_base_link";
    collision_objects[0].id = "tennis_ball";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].SPHERE;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.03;
    collision_objects[0].primitives[0].dimensions[1] = 0.03;
    collision_objects[0].primitives[0].dimensions[2] = 0.03;

    /* Define the pose of the object. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = tennis_position.point.x+0.07;
    collision_objects[0].primitive_poses[0].position.y = tennis_position.point.y+0.05;
    collision_objects[0].primitive_poses[0].position.z = -0.085;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    ROS_INFO("the position of tennis ball: x:%f\n",tennis_position.point.x);
    ROS_INFO("the position of tennis ball: y:%f\n",tennis_position.point.y);
    ROS_INFO("the position of tennis ball: z:%f\n",tennis_position.point.z);

    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface->applyCollisionObjects(collision_objects);
}

/// @brief pick up the tennis ball
bool SimulationController::pick_up(void)
{
    arm_move_group->setPlanningTime(45.0);

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "locobot/arm_base_link";
    tf2::Quaternion orientation;
    // orientation.setRPY(0, 0, 0);
    orientation.setRPY(0, M_PI/2, 0);
    ROS_INFO("=========================================ORIENTATION:%f\n",orientation.getX());
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = tennis_position.point.x+0.05;
    grasps[0].grasp_pose.pose.position.y = tennis_position.point.y+0.05;
    grasps[0].grasp_pose.pose.position.z = 0.03;

    

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "locobot/arm_base_link";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.z = 1;
    grasps[0].pre_grasp_approach.min_distance = -0.265;
    grasps[0].pre_grasp_approach.desired_distance = -0.23;

    // // Setting post-grasp retreat
    // // ++++++++++++++++++++++++++
    // /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "locobot/arm_base_link";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.35;

    openGripper(grasps[0].pre_grasp_posture);

    closedGripper(grasps[0].grasp_posture);

    arm_move_group->pick("tennis_ball", grasps);

    return true;
}

/// @brief the whole command process center
bool SimulationController::Command_Process(interbotix_controller_service::MoveItPlan::Request &req, interbotix_controller_service::MoveItPlan::Response &res)
{
    bool success = false;
    std::string service_type;
    if(req.cmd == interbotix_controller_service::MoveItPlan::Request::CMD_CONTROL_CAMERA_ANGLE)
    {
        success = publish_camera_tilt_angle(req.camera_angle);
        service_type = "Adjust Camera Angle";
    }
    else if(req.cmd == interbotix_controller_service::MoveItPlan::Request::CMD_PLAN_POSE)
    {
        success = plan_end_effector_pose(req.ee_pose);
        service_type = "Planning EE pose";
    }
    else if(req.cmd == interbotix_controller_service::MoveItPlan::Request::CMD_PLAN_JOINT_POSE)
    {
        // float temp[] = req.joint_vector;
        // convert array from service to vector
        std::vector<double> joint_vector(std::begin(req.joint_vector),std::end(req.joint_vector));
        success = plan_joint_positions(joint_vector);
        service_type = "Planning Joint Pose";
    }
    else if (req.cmd == interbotix_controller_service::MoveItPlan::Request::CMD_EXECUTE)
    {
        success = execute_plan();
        service_type = "Execution";
    }
    else if (req.cmd == interbotix_controller_service::MoveItPlan::Request::CMD_PLAN_JOINT_POSE_TO_HOME)
    {
        success = plan_joint_pose_to_home();
        service_type = "Plan to home";
    }
    else if (req.cmd == interbotix_controller_service::MoveItPlan::Request::CMD_PLAN_JOINT_POSE_TO_SLEEP)
    {
        success = plan_joint_pose_to_sleep();
        service_type = "Plan to Sleep";
    }
    else if (req.cmd == interbotix_controller_service::MoveItPlan::Request::CMD_OPEN_GRIPPER)
    {
        success = open_gripper();
        service_type = "Open the gripper";
    }
    else if(req.cmd == interbotix_controller_service::MoveItPlan::Request::CMD_CLOSE_GRIPPER)
    {
        success = close_gripper();
        service_type = "Close the gripper";
    }
    else if (req.cmd == interbotix_controller_service::MoveItPlan::Request::CMD_PICK_UP)
    {
        tennis_position = req.tennis_position;

        ROS_INFO("????????????????????????????the position of tennis ball: x:%f\n",tennis_position.point.x);
        ROS_INFO("????????????????????????????the position of tennis ball: y:%f\n",tennis_position.point.y);
        ROS_INFO("????????????????????????????the position of tennis ball: z:%f\n",tennis_position.point.z);
        
        // add tennis ball as the object to pick up
        add_tennis_ball_as_object();
        // and then pick up it
        success = pick_up();
        service_type = "Pick up tennis ball";
    }
    else if (req.cmd == interbotix_controller_service::MoveItPlan::Request::CMD_PICK_UP_TEST)
    {
        success = pick_up_test();
        service_type = "Testing Pick up tennis ball";
    }
    

    res.success = success;
    if (success)
        res.msg.data = service_type +" was successful!";
    else
        res.msg.data = service_type + " was not successful.";
    
    return true;
}