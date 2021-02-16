//
// Created by zeid on 2/27/20.
//
#include "robot_controller.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id) :
robot_controller_nh_("/ariac/"+arm_id),
robot_controller_options("manipulator",
        "/ariac/"+arm_id+"/robot_description",
        robot_controller_nh_),
robot_move_group_(robot_controller_options) 
{   
    arm_id_ = arm_id;
    ROS_WARN(">>>>> RobotController");

    robot_move_group_.setPlanningTime(3);
    robot_move_group_.setNumPlanningAttempts(1);
    robot_move_group_.setPlannerId("RRTstarkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(1.0);
    robot_move_group_.setMaxAccelerationScalingFactor(0.8);
    robot_move_group_.setGoalPositionTolerance(0.005);
    robot_move_group_.setGoalOrientationTolerance(0.005);
    robot_move_group_.setGoalJointTolerance(0.01);
    // robot_move_group_.setPlanningTime(10);
    // robot_move_group_.setNumPlanningAttempts(3);
    // robot_move_group_.setPlannerId("TRRTkConfigDefault");
    // robot_move_group_.setMaxVelocityScalingFactor(0.9);
    // robot_move_group_.setMaxAccelerationScalingFactor(0.9);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);

    home_joint_pose_bin_ = {0.0, 3.14, -1.26, 2.6, 3.55, -1.60, 0};
    
    home_joint_pose_bin_drop_ = {-1.18, 0.25, -2.14, -2.51, -0.31, -4.65, 0};
    // home_joint_pose_bin_drop_ = {-1.18, 0.25, -1.51, -2.51, -0.55, -4.65, 0};

    //-- The joint positions for the home position to pick from the conveyer belt
    if (arm_id == "arm1"){
        home_joint_pose_conv_ = {-1.0, 3.27, -2.13, -1.76, -0.8, -4.65, 0};
    }
    else{
        home_joint_pose_conv_ = {1.0, 3.27, -2.38, -1.76, -0.55, -4.65, 0};
    }
    

    joint_names_ = {"linear_arm_actuator_joint",  "shoulder_pan_joint", "shoulder_lift_joint", 
    "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    // home_joint_pose_kit1_ = {1.18, 1.51, -1.26, 1.88, 4.02, -1.51, 0};
    home_joint_pose_kit1_ = {1.18, 1.51, -1.00, 2.01, 3.66, -1.51, 0};

    home_joint_pose_kit1_p2_ = {1.18, 1.38, -0.75, 1.51, 3.91, -1.51, 0};

    home_joint_pose_kit2_ = {-1.18, 4.52, -1.51, 2.26, 3.91, -1.51, 0};
    
    home_joint_pose_kit2_p2_ = {-1.18, 4.52, -0.75, 1.51, 3.91, -1.51, 0};

    // home_joint_pose_kit2_ = {-1.18, 4.52, -1.00, 2., 3.66, -1.51, 0};


    // home_joint_pose_kit2_ = {-1.18, 4.52, -1.00, 2., 3.66, -1.51, 0};
    home_arm_1_pose_ = {1.18, 0, -1.51, 0, 2.89, -1.51, 0};

    home_arm_2_pose_ = {-1.18, -3.02, -1.51, 0, 3.28, -1.51, 0};

    part_flip_arm_1_pose_ = {-0.7, 4.65, -2.39, 2.14, 3.39, -1.51, 0};
    part_flip_arm_2_pose_ = {0.7, -1.63, -0.75, -2.14, 6.00, 1.75,0};

    part_exch_arm_1_pose_ = {0.8, 4.4, -1.76, 2.14, 4.25, -1.5, 0};
    part_exch_arm_2_pose_ = {-0.8, 1.38, -1.51, 2.2, 4, -1.63, 0};    

    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker
    offset_ = 0.025;

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/"+arm_id+"/gripper/state", 10, &RobotController::GripperCallback, this);

    SendRobotHome("bin");

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/"+arm_id+"/gripper/control");
    drop_flag_ = false;
}

RobotController::~RobotController() {}

/**
 *
 * @return
 */
bool RobotController::Planner() {
    ROS_INFO_STREAM("Planning started...");
    if (robot_move_group_.plan(robot_planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
    }

    return plan_success_;
}


void RobotController::Execute() {
    ros::AsyncSpinner spinner(6);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }
    spinner.stop();
}

void RobotController::ChangeOrientation(geometry_msgs::Quaternion orientation_target, geometry_msgs::Quaternion orientation_part){

    
    tf::Quaternion Q;
    double roll, pitch, yaw_target, yaw_part,yaw{0};
    tf::quaternionMsgToTF(orientation_target,Q);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw_target);
    tf::quaternionMsgToTF(orientation_part, Q);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw_part);
    ROS_INFO_STREAM("Target yaw --> "<< yaw_target<<", part yaw -->"<< yaw_part);
    if (arm_id_=="arm1"){
        // yaw = (yaw_part-1.6) - yaw_target;
        yaw = yaw_part - yaw_target;
    }
    else{
        // yaw = yaw_target - (yaw_part - 1.57);
        yaw = yaw_part - yaw_target;
    }
    ROS_INFO_STREAM(">>>>> Rotation :"<< yaw);
    ros::AsyncSpinner spinner(6);
    spinner.start();
    
    ROS_INFO_STREAM("Adjusting the orientation");
    robot_move_group_.setJointValueTarget("wrist_3_joint",yaw);
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }

    spinner.stop();

    robot_tf_listener_.waitForTransform(""+arm_id_+"_linear_arm_actuator", ""+arm_id_+"_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/"+arm_id_+"_linear_arm_actuator", "/"+arm_id_+"_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);

    ros::Duration(0.5).sleep();

}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose) {
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(6);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
    spinner.stop();
}

void RobotController::GoToTarget(std::initializer_list<geometry_msgs::Pose> list) {
    ros::AsyncSpinner spinner(6);
    spinner.start();

    // tf::Quaternion myQuaternion;
    // myQuaternion.setRPY(1.57, 1.57, 0);

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        // i.orientation.x = myQuaternion[0];
        // i.orientation.y = myQuaternion[1];
        // i.orientation.z = myQuaternion[2];
        // i.orientation.w = myQuaternion[3];
        waypoints.emplace_back(i);
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    ros::Duration(0.5).sleep();

    robot_planner_.trajectory_ = traj;


    robot_move_group_.execute(robot_planner_);
    ros::Duration(0.5).sleep();
    spinner.stop();
}

void RobotController::SendRobotExch(std::string arm, double buffer){
    
    std::vector<double> temp_pose;
    if (arm=="arm1"){
        temp_pose = part_flip_arm_1_pose_;
        temp_pose[0] -= buffer;
    }
    else{
        temp_pose = part_flip_arm_2_pose_;
        temp_pose[0] += buffer;
    }
    robot_move_group_.setJointValueTarget(temp_pose); 
    ros::AsyncSpinner spinner(6);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }

    spinner.stop();
}

void RobotController::SendRobotHome(std::string pose, double offset) {
    if (pose=="bin") {
        robot_move_group_.setJointValueTarget(home_joint_pose_bin_);
    }
    else if (pose=="kit1"){
        unsigned int i = 0;
        for (const auto &joint_name: joint_names_){
            robot_move_group_.setJointValueTarget(joint_name, home_joint_pose_kit1_[i]);
            i++;
        }
    }
    else if (pose=="kit2"){
        unsigned int i = 0;
        for (const auto &joint_name: joint_names_){
            robot_move_group_.setJointValueTarget(joint_name, home_joint_pose_kit2_[i]);
            i++;
        }
    }
    else if (pose == "kit1_p2"){
        robot_move_group_.setJointValueTarget(home_joint_pose_kit1_p2_);
    }
    else if (pose == "kit2_p2"){
        robot_move_group_.setJointValueTarget(home_joint_pose_kit2_p2_);
    }
    else if (pose=="conv"){
        robot_move_group_.setJointValueTarget(home_joint_pose_conv_);
    }
    else if (pose=="arm1"){
        robot_move_group_.setJointValueTarget(home_arm_1_pose_);
    }
    else if (pose=="arm2"){
        robot_move_group_.setJointValueTarget(home_arm_2_pose_);
    }
    else if (pose=="arm1_exch"){
        // unsigned int i = 0;
        // for (const auto &joint_name: joint_names_){
        //     if (i==0){
        //         i++;
        //         continue;
        //     }
        //     robot_move_group_.setJointValueTarget(joint_name, part_exch_arm_1_pose_[i]);
        //     i++;
        // }
        robot_move_group_.setJointValueTarget(part_exch_arm_1_pose_);
    }
    else if (pose=="arm2_exch"){
        // unsigned int i = 0;
        // for (const auto &joint_name: joint_names_){
        //     if (i==0){
        //         i++;
        //         continue;
        //     }
        //     robot_move_group_.setJointValueTarget(joint_name, part_exch_arm_2_pose_[i]);
        //     i++;
        // }
        robot_move_group_.setJointValueTarget(part_exch_arm_2_pose_);
    }
    else if (pose=="drop_bin"){
        home_joint_pose_bin_drop_[0] += offset;
        robot_move_group_.setJointValueTarget(home_joint_pose_bin_drop_);
    }

    ros::AsyncSpinner spinner(6);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }

    spinner.stop();

    robot_tf_listener_.waitForTransform(""+arm_id_+"_linear_arm_actuator", ""+arm_id_+"_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/"+arm_id_+"_linear_arm_actuator", "/"+arm_id_+"_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);

    // ROS_INFO_STREAM("Roll: " << roll_def_ << "| Pitch: " <<  pitch_def_ << "| Yaw: " << yaw_def_);

    // ros::Duration(0.5).sleep();
    ros::Duration(0.5).sleep();

}

void RobotController::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(0.5).sleep();
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}

bool RobotController::DropPart(geometry_msgs::Pose part_pose, geometry_msgs::Pose pick_pose) {

    drop_flag_ = true;

    ros::spinOnce();
    ROS_INFO_STREAM("Placing phase activated...");

    if (gripper_state_){
        ROS_INFO_STREAM("Moving towards AGV"+arm_id_+"...");
        ChangeOrientation(part_pose.orientation, pick_pose.orientation);
        auto temp_pose = part_pose;
        temp_pose.position.z += 0.1;
        this->GoToTarget({temp_pose, part_pose});
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(false);
   
    }

    drop_flag_ = false;
    return gripper_state_;
}

bool RobotController::DropPart(geometry_msgs::Pose part_pose) {

    drop_flag_ = true;

    ros::spinOnce();
    ROS_INFO_STREAM("Placing phase activated...");

    if (gripper_state_){
        ROS_INFO_STREAM("Moving towards AGV1...");

       auto temp_pose = part_pose;
       temp_pose.position.z += 0.1;
       this->GoToTarget({temp_pose, part_pose});
       ros::Duration(0.5).sleep();
       ros::spinOnce();
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(false);
    }

    drop_flag_ = false;
    return gripper_state_;
}

void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}


bool RobotController::PickPart(geometry_msgs::Pose& part_pose) {


    ROS_INFO_STREAM("Moving to part...");
    part_pose.position.z = part_pose.position.z + offset_;
    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.15;


    this->GoToTarget({temp_pose_1, part_pose});

    ROS_INFO_STREAM("Actuating the gripper..." << part_pose.position.z);
    this->GripperToggle(true);
    ros::spinOnce();
    while (!gripper_state_) {
        part_pose.position.z -= 0.01;
        this->GoToTarget({temp_pose_1, part_pose});
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
    }
    temp_pose_1 = part_pose;
    temp_pose_1.position.x += 0.1;
    temp_pose_1.position.z += 0.5;
    ROS_INFO_STREAM("Going to waypoint...");
    this->GoToTarget(temp_pose_1);
    return gripper_state_;
}

bool RobotController::PickPartFromConv(geometry_msgs::Pose& part_pose, double pick_time) {
    double plan_time = 1;
    double move_time = 1;
    double wait_off = .15;
    double pick_off = .016;

    robot_move_group_.setPlanningTime(plan_time);
    robot_move_group_.setGoalPositionTolerance(0.001);

    tf::Quaternion myQuaternion;
    myQuaternion.setRPY(1.57, 1.57, 0);

    fixed_orientation_.x = myQuaternion[0];
    fixed_orientation_.y = myQuaternion[1];
    fixed_orientation_.z = myQuaternion[2];
    fixed_orientation_.w = myQuaternion[3];

    part_pose.position.z += wait_off;

    ROS_INFO_STREAM("Moving to wait position...");

    this->GoToTarget(part_pose);

    ROS_INFO_STREAM("Waiting for part...");

    while(ros::Time::now().toSec() < (pick_time - plan_time - move_time)){
        ros::spinOnce();
    }

    ROS_INFO_STREAM("Actuating the gripper...");
    this->GripperToggle(true);
    ros::spinOnce();

    part_pose.position.z -= (wait_off-pick_off);

    ROS_INFO_STREAM("Approaching the part...");
    this->GoToTarget(part_pose);

    ros::Duration(.5).sleep();

    robot_move_group_.setPlanningTime(3);
    robot_move_group_.setGoalPositionTolerance(0.005);

    return gripper_state_;
}
