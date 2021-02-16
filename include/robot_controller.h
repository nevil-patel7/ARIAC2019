//
// Created by zeid on 2/27/20.
//

#ifndef SRC_ROBOT_CONTROLLER_H
#define SRC_ROBOT_CONTROLLER_H


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <stdarg.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <initializer_list>
#include <tf/LinearMath/Quaternion.h>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

class RobotController {
public:
    RobotController(std::string arm_id);
    ~RobotController();
    bool Planner();
    void Execute();
    void GoToTarget(std::initializer_list<geometry_msgs::Pose> list);
    void GoToTarget(const geometry_msgs::Pose& pose);
    void SendRobotHome(std::string pose, double offset=0.0);
    void SendRobotExch(std::string arm, double buffer);
    bool DropPart(geometry_msgs::Pose pose);
    bool DropPart(geometry_msgs::Pose pose, geometry_msgs::Pose part_pose);
    void GripperToggle(const bool& state);
    void GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr& grip);
    void GripperStateCheck(geometry_msgs::Pose pose);
    bool PickPart(geometry_msgs::Pose& part_pose);
    bool PickPartFromConv(geometry_msgs::Pose& part_pose,double pick_time);
    void ChangeOrientation(geometry_msgs::Quaternion orientation,geometry_msgs::Quaternion orientation_part);

    bool GetGripperState(){
        return gripper_state_;
    }
    void SetPlanner(std::string planner){
        robot_move_group_.setPlannerId(planner);
    }
    void SetPlanningTime(int time){
        robot_move_group_.setPlanningTime(time);
    }
    void SetPlanningAttempts(int num){
        robot_move_group_.setNumPlanningAttempts(num);
    }

private:
    ros::NodeHandle robot_controller_nh_;
    moveit::planning_interface::MoveGroupInterface::Options robot_controller_options;
    ros::ServiceClient gripper_client_;
    ros::NodeHandle gripper_nh_;
    ros::Subscriber gripper_subscriber_;

    tf::TransformListener robot_tf_listener_;
    tf::StampedTransform robot_tf_transform_;
    tf::TransformListener agv_tf_listener_;
    tf::StampedTransform agv_tf_transform_;

    geometry_msgs::Pose target_pose_;

    moveit::planning_interface::MoveGroupInterface robot_move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

    osrf_gear::VacuumGripperControl gripper_service_;
    osrf_gear::VacuumGripperState gripper_status_;

    std::string object;
    std::string arm_id_;
    bool plan_success_;
    std::vector<double> home_joint_pose_conv_;
    std::vector<double> home_joint_pose_bin_;
    std::vector<double> home_joint_pose_kit1_;
    std::vector<double> home_joint_pose_kit2_;
    std::vector<double> home_joint_pose_kit1_p2_;
    std::vector<double> home_joint_pose_kit2_p2_;
    std::vector<double> home_arm_1_pose_;
    std::vector<double> home_arm_2_pose_;
    std::vector<double> part_flip_arm_1_pose_;
    std::vector<double> part_flip_arm_2_pose_;
    std::vector<double> part_exch_arm_1_pose_;
    std::vector<double> part_exch_arm_2_pose_;
    std::vector<double> home_joint_pose_bin_drop_;
    std::vector<std::string> joint_names_;
    geometry_msgs::Pose home_cart_pose_;
    geometry_msgs::Quaternion fixed_orientation_;
    geometry_msgs::Pose agv_position_;
    std::vector<double> end_position_;
    double offset_;
    double roll_def_,pitch_def_,yaw_def_;
    tf::Quaternion q;
    int counter_;
    bool gripper_state_, drop_flag_;
};
#endif //SRC_ROBOT_CONTROLLER_H
