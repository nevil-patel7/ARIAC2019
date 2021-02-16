//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>



AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);

}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);

}


/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type, bool conv=false, bool check=true) {
    //--Grab the last one from the list then remove it
    if (conv){
        if (!product_frame_list_conv_.empty() && product_frame_list_conv_.find(product_type)!=product_frame_list_conv_.end()) {
            std::string frame = product_frame_list_conv_[product_type].back();
            ROS_INFO_STREAM("Frame >>>> " << frame);
            product_frame_list_conv_[product_type].pop_back();
            return frame;
        } else {
            return "-1";
        }
    }
    else if (!check){
        if (!products_list_.empty() && products_list_.find(product_type)!=products_list_.end()) {
            std::string frame = products_list_[product_type].back().first;
            ROS_INFO_STREAM("Frame >>>> " << frame);
            products_list_[product_type].pop_back();
            return frame;
        } else {
            return "-1";
        }
    }
    else {
        if (!products_check_list_.empty() && products_check_list_.find(product_type)!=products_check_list_.end()) {
            if ( !products_check_list_[product_type].empty()){
                std::string frame = products_check_list_[product_type].back().first;
                ROS_INFO_STREAM("Frame >>>> " << frame);
                products_check_list_[product_type].pop_back();
                return frame;
            } else {
                return "-1";
            }
        } else {
            return "-1";
        }
    }
}

std::pair<std::string, geometry_msgs::Pose> AriacOrderManager::GetProductPair(std::string product_type){

    std::pair<std::string, geometry_msgs::Pose> product_pair;
    product_pair = products_list_[product_type].back();
    products_list_[product_type].pop_back();
    return product_pair;

}

geometry_msgs::Pose AriacOrderManager::FlipPartPickUp(std::string product_type, std::string product_frame, geometry_msgs::Pose part_pose, int agv_id, bool conv){

    int bin_number;
    if (!conv){
        std::string product_part_frame = product_frame;
        std::string delimiter = "_";
        unsigned int pos = 0;
        unsigned int count = 0;
        std::string token;
        while ((pos = product_part_frame.find(delimiter)) != std::string::npos) {
            token = product_part_frame.substr(0, pos);
            count++;
            product_part_frame.erase(0, pos + 1);
            if (count == 3){
                break;
            }
        }
        bin_number = std::stoi(token,nullptr,0);
    }
    else{
        bin_number = 3;
    }

    geometry_msgs::Pose rackDrop;
    rackDrop.position.x = 0.3;
    rackDrop.position.y = 0.0;
    rackDrop.position.z = 0.950;
    rackDrop.orientation.x = 0;
    rackDrop.orientation.y = 0;
    rackDrop.orientation.z = 0;
    rackDrop.orientation.z = 1;
    if(product_type == "pulley_part")
        part_pose.position.z += 0.08;
        rackDrop.position.z = 1.2;
    double offset = part_pose.position.z*0.05;
    bool failed_pick;
    if (agv_id ==1){
        
        if (bin_number<=3){
            arm1_.SendRobotExch("arm1",-0.5);
            bool failed_pick = arm2_.PickPart(part_pose);
            // arm2_.SendRobotHome("bin");
            ROS_WARN_STREAM("Picking up state "<< failed_pick);
            ros::Duration(1).sleep();
            while(!failed_pick){
                failed_pick = arm2_.PickPart(part_pose);
            }
            arm2_.SendRobotExch("arm2",-0.5);
            // arm2_.SendRobotExch("arm2",offset*-1);
            arm1_.SendRobotExch("arm1",0.0);
            arm1_.GripperToggle(true);
            while (!arm1_.GetGripperState()) {
                arm2_.SendRobotExch("arm2",offset);
                ROS_INFO_STREAM("Actuating the gripper...");
                arm1_.GripperToggle(true);
                ros::spinOnce();
                offset += 0.01;
            }
            arm2_.GripperToggle(false);
            arm1_.SendRobotExch("arm1",-0.5);
            arm2_.SendRobotExch("arm2",-0.5);
            arm2_.SendRobotHome("bin");
            rackDrop = part_pose;
        }

        else{
            bool failed_pick = arm1_.PickPart(part_pose);
            arm2_.SendRobotHome("arm2_exch");
            ROS_WARN_STREAM("Picking up state "<< failed_pick);
            ros::Duration(1).sleep();
            while(!failed_pick){
                failed_pick = arm1_.PickPart(part_pose);
            }
            arm1_.SendRobotHome("arm1_exch");
            bool drop = arm1_.DropPart(rackDrop);
            arm1_.SendRobotHome("arm1_exch");
            rackDrop.position.z = 0.955;
            if (product_type == "pulley_part"){
                rackDrop.position.z += 0.08;
            }
            double r=0, p=0, y=1.57;
            tf2::Quaternion q_rot, q_new, q_old;
            q_rot.setRPY(r,p,y);
            q_old[0] = part_pose.orientation.x;
            q_old[1] = part_pose.orientation.y;
            q_old[2] = part_pose.orientation.z;
            q_old[3] = part_pose.orientation.w;
            q_new = q_rot * q_old;
            q_new.normalize();
            rackDrop.orientation.x = q_new[0];
            rackDrop.orientation.y = q_new[1];
            rackDrop.orientation.z = q_new[2];
            rackDrop.orientation.w = q_new[3];
            
            failed_pick = arm2_.PickPart(rackDrop);
            while(!failed_pick){
                failed_pick = arm2_.PickPart(rackDrop);
            }
            arm1_.SendRobotExch("arm1",-0.5);
            ROS_WARN_STREAM("Picking up state "<< failed_pick);
            ros::Duration(1).sleep();
            arm2_.SendRobotExch("arm2",-0.5);
            // arm2_.SendRobotExch("arm2",offset);
            arm1_.SendRobotExch("arm1",0.0);
            arm1_.GripperToggle(true);
            while (!arm1_.GetGripperState()) {
                arm2_.SendRobotExch("arm2",offset);
                ROS_INFO_STREAM("Actuating the gripper...");
                arm1_.GripperToggle(true);
                ros::spinOnce();
                offset += 0.01;
            }
            arm2_.GripperToggle(false);
            arm1_.SendRobotExch("arm1",-0.5);
            arm2_.SendRobotExch("arm2",-0.5);
            arm2_.SendRobotHome("bin");
        }
    }

    else {
        if (bin_number>=3){
            arm2_.SendRobotExch("arm2",-0.5);
            bool failed_pick = arm1_.PickPart(part_pose);
            // arm1_.SendRobotHome("bin");
            ROS_WARN_STREAM("Picking up state "<< failed_pick);
            ros::Duration(1).sleep();
            while(!failed_pick){
                failed_pick = arm1_.PickPart(part_pose);
            }
            arm1_.SendRobotExch("arm1",-0.5);
            // arm1_.SendRobotExch("arm1",offset*-1);
            arm2_.SendRobotExch("arm2",0.0);
            arm2_.GripperToggle(true);
            while (!arm2_.GetGripperState()) {
                arm1_.SendRobotExch("arm1",offset);
                ROS_INFO_STREAM("Actuating the gripper...");
                arm2_.GripperToggle(true);
                ros::spinOnce();
                offset += 0.01;
            }
            arm1_.GripperToggle(false);
            arm1_.SendRobotExch("arm1",-0.5);
            arm2_.SendRobotExch("arm2",-0.5);
            arm1_.SendRobotHome("bin");
            rackDrop = part_pose;
        }

        else{
            bool failed_pick = arm2_.PickPart(part_pose);
            arm1_.SendRobotHome("arm1_exch");
            ROS_WARN_STREAM("Picking up state "<< failed_pick);
            ros::Duration(1).sleep();
            while(!failed_pick){
                failed_pick = arm2_.PickPart(part_pose);
            }
            arm2_.SendRobotHome("arm2_exch");
            bool drop = arm2_.DropPart(rackDrop);
            arm2_.SendRobotHome("arm2_exch");
            rackDrop.position.z = 0.955;
            if (product_type == "pulley_part"){
                rackDrop.position.z += 0.08;
            }
            double r=0, p=0, y=3.14;
            tf2::Quaternion q_rot, q_new, q_old;
            q_rot.setRPY(r,p,y);
            q_old[0] = part_pose.orientation.x;
            q_old[1] = part_pose.orientation.y;
            q_old[2] = part_pose.orientation.z;
            q_old[3] = part_pose.orientation.w;
            q_new = q_rot * q_old;
            q_new.normalize();
            rackDrop.orientation.x = q_new[0];
            rackDrop.orientation.y = q_new[1];
            rackDrop.orientation.z = q_new[2];
            rackDrop.orientation.w = q_new[3];

            failed_pick = arm1_.PickPart(rackDrop);
            while(!failed_pick){
                failed_pick = arm1_.PickPart(rackDrop);
            }

            arm2_.SendRobotExch("arm2",-0.5);
            ROS_WARN_STREAM("Picking up state "<< failed_pick);
            ros::Duration(1).sleep();
            arm1_.SendRobotExch("arm1",-0.5);
            // arm1_.SendRobotExch("arm1",offset*-1);
            arm2_.SendRobotExch("arm2",0.0);
            arm2_.GripperToggle(true);
            while (!arm2_.GetGripperState()) {
                arm1_.SendRobotExch("arm1",offset);
                ROS_INFO_STREAM("Actuating the gripper...");
                arm2_.GripperToggle(true);
                ros::spinOnce();
                offset += 0.01;
            }
            arm1_.GripperToggle(false);
            arm1_.SendRobotExch("arm1",-0.5);
            arm2_.SendRobotExch("arm2",-0.5);
            arm1_.SendRobotHome("bin");

            
        }
    }

    return rackDrop;

}

geometry_msgs::Pose AriacOrderManager::PickUp(std::string product_type, std::string product_frame, geometry_msgs::Pose part_pose, int agv_id, bool conv){

    int bin_number;
    if (!conv){
        std::string product_part_frame = product_frame;
        std::string delimiter = "_";
        unsigned int pos = 0;
        unsigned int count = 0;
        std::string token;
        while ((pos = product_part_frame.find(delimiter)) != std::string::npos) {
            token = product_part_frame.substr(0, pos);
            count++;
            product_part_frame.erase(0, pos + 1);
            if (count == 3){
                break;
            }
        }
        bin_number = std::stoi(token,nullptr,0);
    }
    else{
        bin_number = 3;
    }
    // auto part_pose = camera_.GetPartPose("/world", product_frame);
    
    bool failed_pick;
    geometry_msgs::Pose rackDrop;
    rackDrop.position.x = 0.3;
    rackDrop.position.y = 0.0;
    rackDrop.position.z = 0.70;
    rackDrop.orientation.x = 0;
    rackDrop.orientation.y = 0;
    rackDrop.orientation.z = 0;
    rackDrop.orientation.z = 1;
    if(product_type == "pulley_part")
        part_pose.position.z += 0.08;
        rackDrop.position.z = 1.2;
    if (agv_id ==1){
        
        if (bin_number<=2){
            bool failed_pick = arm2_.PickPart(part_pose);
            arm1_.SendRobotHome("arm1_exch");
            ROS_WARN_STREAM("Picking up state "<< failed_pick);
            // ros::Duration(1).sleep();
            while(!failed_pick){
                failed_pick = arm2_.PickPart(part_pose);
            }
            arm2_.SendRobotHome("arm2_exch");
            bool drop = arm2_.DropPart(rackDrop);
            arm2_.SendRobotHome("bin");
            rackDrop.position.z = 0.955;
            if (product_type == "pulley_part"){
                rackDrop.position.z += 0.08;
            }
            double r=0, p=0, y=3.14;
            tf2::Quaternion q_rot, q_new, q_old;
            q_rot.setRPY(r,p,y);
            q_old[0] = part_pose.orientation.x;
            q_old[1] = part_pose.orientation.y;
            q_old[2] = part_pose.orientation.z;
            q_old[3] = part_pose.orientation.w;
            q_new = q_rot * q_old;
            q_new.normalize();
            rackDrop.orientation.x = q_new[0];
            rackDrop.orientation.y = q_new[1];
            rackDrop.orientation.z = q_new[2];
            rackDrop.orientation.w = q_new[3];

            failed_pick = arm1_.PickPart(rackDrop);
            while(!failed_pick){
                failed_pick = arm1_.PickPart(rackDrop);
            }
        }

        else{
            bool failed_pick = arm1_.PickPart(part_pose);
            while(!failed_pick){
                failed_pick = arm1_.PickPart(part_pose);
            }
            rackDrop = part_pose;
        }
    }

    else {
        if (bin_number>=4){
            bool failed_pick = arm1_.PickPart(part_pose);
            arm2_.SendRobotHome("arm2_exch");
            ROS_WARN_STREAM("Picking up state "<< failed_pick);
            // ros::Duration(1).sleep();
            while(!failed_pick){
                failed_pick = arm1_.PickPart(part_pose);
            }
            arm1_.SendRobotHome("arm1_exch");
            bool drop = arm1_.DropPart(rackDrop);
            arm1_.SendRobotHome("bin");
            rackDrop.position.z = 0.955;
            if (product_type == "pulley_part"){
                rackDrop.position.z += 0.08;
            }
            double r=0, p=0, y=1.57;
            tf2::Quaternion q_rot, q_new, q_old;
            q_rot.setRPY(r,p,y);
            q_old[0] = part_pose.orientation.x;
            q_old[1] = part_pose.orientation.y;
            q_old[2] = part_pose.orientation.z;
            q_old[3] = part_pose.orientation.w;
            q_new = q_rot * q_old;
            q_new.normalize();
            rackDrop.orientation.x = q_new[0];
            rackDrop.orientation.y = q_new[1];
            rackDrop.orientation.z = q_new[2];
            rackDrop.orientation.w = q_new[3];
            
            failed_pick = arm2_.PickPart(rackDrop);
            while(!failed_pick){
                failed_pick = arm2_.PickPart(rackDrop);
            }
        }

        else{
            bool failed_pick = arm2_.PickPart(part_pose);
            while(!failed_pick){
                failed_pick = arm2_.PickPart(part_pose);
            }
            rackDrop = part_pose;
        }
    }

    return rackDrop;

}

std::string AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    
    arm1_.SendRobotHome("bin");
    arm2_.SendRobotHome("bin");

    std::string product_type = product_type_pose.first;
    ROS_WARN_STREAM("Product type >>>> " << product_type);
    auto product_pair = this->GetProductPair(product_type);
    std::string product_frame = product_pair.first;
    auto part_pose = product_pair.second;
    // std::string product_frame = this->GetProductFrame(product_type, false,false);
    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
    
    // auto part_pose = camera_.GetPartPose("/world",product_frame);

    tf::Quaternion Q;
    double roll, pitch, yaw, rollReq, rollPart;
    tf::quaternionMsgToTF(product_type_pose_.second.orientation,Q);
    tf::Matrix3x3(Q).getRPY(rollReq,pitch,yaw);
    tf::quaternionMsgToTF(part_pose.orientation,Q);
    tf::Matrix3x3(Q).getRPY(rollPart,pitch,yaw);
    roll = rollPart - rollReq;
    ROS_WARN_STREAM("Roll is ->>>>> "<<roll);
    if (roll>=3 || roll<=-3){
        part_pose = this->FlipPartPickUp(product_type, product_frame, part_pose, agv_id, false);
    }
    else{
        part_pose = this->PickUp(product_type, product_frame, part_pose, agv_id, false);
    }

    
    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    if(agv_id==1){
        arm1_.SendRobotHome("kit1");
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.orientation.x <<","<< StampedPose_out.pose.orientation.y 
                        << "," << StampedPose_out.pose.orientation.z<< "," << StampedPose_out.pose.orientation.w<<")");
        auto temp_frame = product_frame;
        temp_frame[15] = '8';
        
        ros::Duration(1.0).sleep();
        if (arm1_.GetGripperState()){
            arm1_.SendRobotHome("kit1_p2");
            auto temp_pose = camera_.GetPartPose("/world",temp_frame);
            if(temp_pose.orientation.z!=100 && temp_pose.orientation.w!=100){
                part_pose = temp_pose;
            }
        } else {
            part_pose = camera_.GetPartPose("/world",temp_frame);
            if (part_pose.orientation.z!=100 && part_pose.orientation.w!=100){
                auto temp_pose = part_pose;
                temp_pose.position.z += 0.15;
                part_pose.position.z += 0.025;
                if (product_type == "pulley_part"){ part_pose.position.z +=0.08;}
                arm1_.GoToTarget({temp_pose,part_pose});
                arm1_.GripperToggle(true);
                bool failed_pick = arm1_.GetGripperState();
                while(!failed_pick){
                    part_pose.position.z -= 0.01;
                    arm1_.GoToTarget(part_pose);
                    arm1_.GripperToggle(true);
                    ros::spinOnce();
                    failed_pick = arm1_.GetGripperState();
                }
                arm1_.GoToTarget(temp_pose);
                // arm1_.SendRobotHome("kit1");
                // ros::Duration(1.0).sleep();
                // part_pose = camera_.GetPartPose("/world",temp_frame);
            } else {
                arm1_.SendRobotHome("kit1");
                return temp_frame;
            }
        }
        parts_list_kit_1_.push_back(temp_frame);
    }
    else{
        arm2_.SendRobotHome("kit2");
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.orientation.x <<","<< StampedPose_out.pose.orientation.y 
                        << "," << StampedPose_out.pose.orientation.z<< "," << StampedPose_out.pose.orientation.w<<")");
        auto temp_frame = product_frame;
        temp_frame[15] = '9';
        
        arm2_.SendRobotHome("kit2_p2");
        ros::Duration(1.0).sleep();
        if (arm2_.GetGripperState()){
            auto temp_pose = camera_.GetPartPose("/world",temp_frame);
            if(temp_pose.orientation.z!=100 && temp_pose.orientation.w!=100){
                part_pose = temp_pose;
            }
        }
        else{
            part_pose = camera_.GetPartPose("/world",temp_frame);
            if (part_pose.orientation.z!=100 && part_pose.orientation.w!=100){
                auto temp_pose = part_pose;
                temp_pose.position.z += 0.15;
                part_pose.position.z += 0.025;
                if (product_type == "pulley_part"){ part_pose.position.z +=0.08;}
                arm2_.GoToTarget({temp_pose,part_pose});
                arm2_.GripperToggle(true);
                bool failed_pick = arm2_.GetGripperState();
                while(!failed_pick){
                    part_pose.position.z -= 0.01;
                    arm2_.GoToTarget(part_pose);
                    arm2_.GripperToggle(true);
                    bool failed_pick = arm2_.GetGripperState();
                }
                // arm2_.SendRobotHome("kit2_p2");
                arm2_.GoToTarget(temp_pose);
                
            }
            else {
                arm2_.SendRobotHome("kit2");
                return temp_frame;
            }
        }
        parts_list_kit_2_.push_back(temp_frame);

    }
 
    if (agv_id==1){
        auto result = arm1_.DropPart(StampedPose_out.pose, part_pose);
    }
    else{
        auto result = arm2_.DropPart(StampedPose_out.pose, part_pose);
    }

    if (agv_id==1){
        arm1_.SendRobotHome("kit1");
    }
    else{
        arm2_.SendRobotHome("kit2");
    }

    return product_frame;
}


bool AriacOrderManager::PickAndPlaceFromConv(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    
    // if (agv_id==1){
    arm1_.SendRobotHome("bin");
        // arm2_.SendRobotHome("arm2");
    // }
    // else {
        // arm1_.SendRobotHome("arm1");
    arm2_.SendRobotHome("bin");
    // }
    
    geometry_msgs::Pose part_pose;
    std::string product_frame;
    if (!products_list_conv_.empty() && products_list_conv_.find(product_type_pose.first)!=products_list_conv_.end() && !products_list_conv_[product_type_pose.first].empty()){
        auto product_pair = products_list_conv_[product_type_pose.first].back();
        product_frame = product_pair.first;
        part_pose = product_pair.second;
        products_list_conv_[product_type_pose.first].pop_back();
    } 
    else {
        return false;
    }

    tf::Quaternion Q;
    double roll, pitch, yaw, rollReq, rollPart;
    tf::quaternionMsgToTF(product_type_pose_.second.orientation,Q);
    tf::Matrix3x3(Q).getRPY(rollReq,pitch,yaw);
    tf::quaternionMsgToTF(part_pose.orientation,Q);
    tf::Matrix3x3(Q).getRPY(rollPart,pitch,yaw);
    roll = rollPart - rollReq;
    ROS_WARN_STREAM("Roll is ->>>>> "<<roll);
    if (roll>=2 || roll<=-2){
        part_pose = this->FlipPartPickUp(product_type_pose.first, product_frame, part_pose, agv_id, true);
    }
    else{
        part_pose = this->PickUp(product_type_pose.first, product_frame, part_pose, agv_id, true);
    }
    // if(product_type_pose.first == "pulley_part")
    //     part_pose.position.z += 0.08;
    // if (agv_id == 1){
    //     bool failed_pick = arm1_.PickPart(part_pose);
    //     while(!failed_pick){
    //         failed_pick = arm1_.PickPart(part_pose);
    //     }
    // }
    // else {
    //     bool failed_pick = arm2_.PickPart(part_pose);
    //     while(!failed_pick){
    //         failed_pick = arm2_.PickPart(part_pose);
    //     }
    // }
    
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    if(agv_id==1){
        arm1_.SendRobotHome("kit1");
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.orientation.x <<","<< StampedPose_out.pose.orientation.y 
                        << "," << StampedPose_out.pose.orientation.z<< "," << StampedPose_out.pose.orientation.w<<")");
        auto temp_frame = product_frame;
        temp_frame[15] = '8';

        arm1_.SendRobotHome("kit1_p2");
        ros::Duration(1.0).sleep();
        if (arm1_.GetGripperState()){
            auto temp_pose = camera_.GetPartPose("/world",temp_frame);
            if(temp_pose.orientation.z!=100 && temp_pose.orientation.w!=100){
                part_pose = temp_pose;
            }
        }
        else{
            part_pose = camera_.GetPartPose("/world",temp_frame);
            if (part_pose.orientation.z!=100 && part_pose.orientation.w!=100){
                auto temp_pose = part_pose;
                temp_pose.position.z += 0.15;
                part_pose.position.z += 0.025;
                arm1_.GoToTarget({temp_pose,part_pose});
                arm1_.GripperToggle(true);
                bool failed_pick = arm1_.GetGripperState();
                while(!failed_pick){
                    part_pose.position.z -= 0.01;
                    arm1_.GoToTarget(part_pose);
                    arm1_.GripperToggle(true);
                    bool failed_pick = arm1_.GetGripperState();
                }
                arm1_.GoToTarget(temp_pose);
            }
            else{
                arm1_.SendRobotHome("kit1");
                return false;
            }
        }
        parts_list_kit_1_.push_back(temp_frame);

    }
    else{
        arm2_.SendRobotHome("kit2");
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.orientation.x <<","<< StampedPose_out.pose.orientation.y 
                        << "," << StampedPose_out.pose.orientation.z<< "," << StampedPose_out.pose.orientation.w<<")");
        auto temp_frame = product_frame;
        temp_frame[15] = '9';

        arm2_.SendRobotHome("kit2_p2");
        ros::Duration(1.0).sleep();
        if (arm2_.GetGripperState()){
            auto temp_pose = camera_.GetPartPose("/world",temp_frame);
            if(temp_pose.orientation.z!=100 && temp_pose.orientation.w!=100){
                part_pose = temp_pose;
            }
        }
        else{
            part_pose = camera_.GetPartPose("/world",temp_frame);
            if (part_pose.orientation.z!=100 && part_pose.orientation.w!=100){
                auto temp_pose = part_pose;
                temp_pose.position.z += 0.15;
                part_pose.position.z += 0.025;
                arm2_.GoToTarget({temp_pose,part_pose});
                arm2_.GripperToggle(true);
                bool failed_pick = arm2_.GetGripperState();
                while(!failed_pick){
                    part_pose.position.z -= 0.01;
                    arm2_.GoToTarget(part_pose);
                    arm2_.GripperToggle(true);
                    bool failed_pick = arm2_.GetGripperState();
                }
                arm2_.GoToTarget(temp_pose);
            }
            else{
                arm2_.SendRobotHome("kit2");
                return false;
            }
        }
        parts_list_kit_2_.push_back(temp_frame);
        
    }
    bool result;
    if (agv_id==1){
        auto result = arm1_.DropPart(StampedPose_out.pose, part_pose);
    }
    else{
        auto result = arm2_.DropPart(StampedPose_out.pose, part_pose);
    }

    if (agv_id==1){
        arm1_.SendRobotHome("kit1");
    }
    else{
        arm2_.SendRobotHome("kit2");
    }

    return result;
}



bool AriacOrderManager::CheckOrderUpdate(int current_order_count, std::string orderID){

    ROS_WARN_STREAM("received_orders size "<< received_orders_.size());
    
    if (current_order_count+1 == received_orders_.size()){
        return false;
    }

    else {
        ROS_WARN_STREAM("received_orders last id "<< received_orders_[current_order_count+1].order_id <<" , "
        << "current order id "<< orderID);
        if(orderID[6] == received_orders_[current_order_count+1].order_id[6]){
            return true;
        }
    }

    return false;

}

void AriacOrderManager::ClearTray(int agv_id){

    if (agv_id == 1){
        while(parts_list_kit_1_.size()!=0){
            std::string frame = parts_list_kit_1_.back();
            std::string product_type = frame.substr(17,6);
            auto part_pose = camera_.GetPartPose("/world",frame);
            if(product_type == "pulley")
                part_pose.position.z += 0.08;
            //--task the robot to pick up this part
            bool failed_pick;
            failed_pick = arm1_.PickPart(part_pose);
            ROS_WARN_STREAM("Picking up state " << failed_pick);
            ros::Duration(1).sleep();
            while(!failed_pick){
                auto part_pose = camera_.GetPartPose("/world",frame);
                failed_pick = arm1_.PickPart(part_pose);
            }
            part_pose.position.z += 0.3;
            part_pose.position.x = 0.3;
            part_pose.position.y = 2.5;
            bool result;
            result = arm1_.DropPart(part_pose);
            parts_list_kit_1_.pop_back();
        }
    }
    else{
        while(parts_list_kit_2_.size()!=0){
            std::string frame = parts_list_kit_2_.back();
            std::string product_type = frame.substr(17,6);
            auto part_pose = camera_.GetPartPose("/world",frame);
            if(product_type == "pulley")
                part_pose.position.z += 0.08;
            //--task the robot to pick up this part
            bool failed_pick;
            failed_pick = arm2_.PickPart(part_pose);
            ROS_WARN_STREAM("Picking up state " << failed_pick);
            ros::Duration(1).sleep();
            while(!failed_pick){
                auto part_pose = camera_.GetPartPose("/world",frame);
                failed_pick = arm2_.PickPart(part_pose);
            }
            part_pose.position.z += 0.3;
            part_pose.position.y = -2.5;
            part_pose.position.x = 0.3;
            bool result;
            result = arm2_.DropPart(part_pose);
            parts_list_kit_2_.pop_back();
        }
    }

}

void AriacOrderManager::PickFromConv2(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id){

    if (agv_id==1){
        arm1_.SendRobotHome("conv");
    }
    else{
        arm2_.SendRobotHome("conv");
    }
    
    int part_num{0};
    geometry_msgs::Pose part_pose;
    ros::Duration timeout(10);
    // ros::Time start_time = ros::Time::now();
    bool pick_state = false;
    double pick_time;

    std::string product_type = product_type_pose.first;
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    ROS_WARN_STREAM("Product type >>>> " << product_type);
    
    while(!pick_state){
        ros::spinOnce();
        bool part_in_list = false;
        ros::Time start_time = ros::Time::now();
        while(!part_in_list){
            ros::spinOnce();
            products_conveyor_list_ = camera_.get_conveyor_list();
            for (const auto &product: products_conveyor_list_){
                if(std::get<0>(product)==product_type){
                    part_in_list = true;
                    pick_time = std::get<1>(product);
                    part_pose = std::get<2>(product);
                    conv_part_num_ = std::get<3>(product);
                    double waitTime = pick_time - ros::Time::now().toSec();
                    ROS_INFO_STREAM("The part is on the conveyor. Picking up in " << waitTime << " seconds.");
                    break;
                }
            }

            if (ros::Time::now() - start_time > timeout) {
                ROS_WARN_STREAM("The desired part is not on the conveyor");
                return;
            }
        }

        auto temp_pose = part_pose;

        if (agv_id==1)
            pick_state = arm1_.PickPartFromConv(temp_pose,pick_time);

        else
            pick_state = arm2_.PickPartFromConv(temp_pose,pick_time);


        if (pick_state) {
            ROS_INFO_STREAM("The part was picked up successfully");
            temp_pose.position.z += .5;
            if (agv_id==1)
                arm1_.GoToTarget(temp_pose);
            else
                arm2_.GoToTarget(temp_pose);
        }

        else {
            ROS_WARN_STREAM("The part was not picked up successfully. Trying again.");
            if (agv_id==1){
                arm1_.GripperToggle(false);
                arm1_.SendRobotHome("conv");
            }
            else {
                arm2_.GripperToggle(false);
                arm2_.SendRobotHome("conv");
            }
        }
    }

    
    if (agv_id==1){
        ROS_INFO_STREAM("Going to drop bin position....");
        arm1_.SendRobotHome("drop_bin");
        arm1_.GripperToggle(false);
    }
    else {
        arm2_.SendRobotHome("drop_bin");
        arm2_.GripperToggle(false);
    }
}

void AriacOrderManager::ConvCollect(int current_order_count) {

    std::vector<std::string> conveyer_parts;
    products_check_list_ = camera_.get_product_frame_list();
    products_list_ = camera_.get_product_frame_list();
    products_conveyor_list_ = camera_.get_conveyor_list();


    auto order = received_orders_[current_order_count];
    auto shipments = order.shipments;
    int i{0};
    for (const auto &shipment: shipments) {
        auto shipment_type = shipment.shipment_type;
        auto agv = shipment.agv_id.back();
        int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
        auto products = shipment.products;
        // ROS_INFO_STREAM("Order ID: " << order_id);
        // ROS_INFO_STREAM("Shipment Type: " << shipment_type);
        // ROS_INFO_STREAM("AGV ID: " << agv_id);
        for (const auto &product: products){
            ros::spinOnce();
            // ROS_INFO_STREAM("Product type: " << product.type);
            if (this->GetProductFrame(product.type)=="-1"){
                conveyer_parts.push_back(product.type);
                if(i==0){
                    arm2_.SendRobotHome("arm2");
                    i++;
                }
            }
        }

    }
    // geometry_msgs::Pose drop_bin_pose;
    // drop_bin_pose.position.x = -0.203;
    // drop_bin_pose.position.y = -0.179;
    // drop_bin_pose.position.z = 1.0;
    // drop_bin_pose.orientation.x = 0;
    // drop_bin_pose.orientation.y = 0;
    // drop_bin_pose.orientation.z = 0;
    // drop_bin_pose.orientation.w = 1;

    std::string lastPart = "";
    int part_num{0};
    std::string product_frame;
    geometry_msgs::Pose conv_part_pose;
    for (const auto &product: conveyer_parts){
        ROS_INFO_STREAM("Picking parts from the conveyer");
        product_type_pose_.first = product;
        // product_type_pose_.second = drop_bin_pose;
        // if (lastPart!=product){
        //     conv_part_pose = PickFromConv(product_type_pose_, 1, 0.0);
        //     if (conv_part_pose.position.x == 0 && conv_part_pose.position.y == 0 && conv_part_pose.position.z == 0){
        //         continue;
        //     }
        // }
        // else{
        //     PickFromConv(product_type_pose_, 1, conv_part_pose, 0.0);
        // }
        PickFromConv2(product_type_pose_,1);
        part_num = conv_part_num_ - 1;
        product_frame = "logical_camera_3_"+product+"_"+std::to_string(part_num)+"_frame";
        product_type_pose_.first = product_frame;
        product_type_pose_.second = camera_.GetPartPose("/world",product_frame);
        products_list_conv_[product].emplace_back(product_type_pose_);
        product_type_pose_.second.position.z += 0.3;
        arm1_.GoToTarget(product_type_pose_.second);
        // drop_bin_pose.position.y -= 0.2;
        lastPart = product;
    }

}

void AriacOrderManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");
    //-- used to check if pick and place was successful
    bool pick_n_place_success{false};
    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;

    ros::Duration(2.0).sleep();
    ros::spinOnce();
    ros::Duration(2.0).sleep();
    products_check_list_ = camera_.get_product_frame_list();
    products_list_ = camera_.get_product_frame_list();
    int current_order_count = 0;
    // Reading the order
    
    while(received_orders_.size()!=current_order_count){
        auto order = received_orders_[current_order_count];
        auto order_id = order.order_id;
        auto shipments = order.shipments;
        ConvCollect(current_order_count);
        products_check_list_ = camera_.get_product_frame_list();
        products_list_ = camera_.get_product_frame_list();
        for (const auto &shipment: shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();
            //--this returns a char
            //-- if agv is any then we use AGV1, else we convert agv id to int
            //--agv-'0' converts '1' to 1 and '2' to 2
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);
            for (const auto &product: products){
                arm1_.SetPlanningTime(5);
                arm2_.SetPlanningTime(5);
                arm1_.SetPlanningAttempts(3);
                arm2_.SetPlanningAttempts(3);
                ros::spinOnce();
                product_type_pose_.first = product.type;
                ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                product_type_pose_.second = product.pose;
                ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
                if (this->GetProductFrame(product_type_pose_.first)=="-1"){
                    pick_n_place_success = PickAndPlaceFromConv(product_type_pose_, agv_id);
                }
                else{
                    bool result = true;
                    while(result){
                        std::string picked_part_frame =  PickAndPlace(product_type_pose_, agv_id);
                        ros::Duration(1).sleep();
                        ros::spinOnce();
                        result = RemoveFailureParts(agv_id, product_type_pose_.first);
                    }
                    if (CheckOrderUpdate(current_order_count, order_id)){
                        ROS_WARN_STREAM(">>>>>>>>Order Update is triggered<<<<<<<<<");
                        ClearTray(agv_id);
                        break;
                    }
                }
                
            }
            // --todo: What do we do if pick and place fails?
            products_check_list_ = products_list_;
            if(agv_id==1){
                arm1_.SendRobotHome("arm1");
            }
            else {
                arm2_.SendRobotHome("arm2");
            }

            //-- Check for order update
            if (!CheckOrderUpdate(current_order_count, order_id)){
                ros::Duration(6).sleep();
                ros::spinOnce();
            }
            if (CheckOrderUpdate(current_order_count, order_id)){
                ROS_WARN_STREAM(">>>>>>>>Order Update is triggered<<<<<<<<<");
                if(agv_id==1){
                    if (parts_list_kit_1_.size()>0){
                        ClearTray(agv_id);
                    }
                }
                else{
                    if (parts_list_kit_2_.size()>0){
                        ClearTray(agv_id);
                    }
                }
                break;
            }
            else{
                ROS_INFO_STREAM("Shipment id: "<< shipment_type);
                SubmitAGV(agv_id, shipment_type);
                ROS_INFO_STREAM("Submitting AGV 1");
                if(agv_id==1){
                    parts_list_kit_1_.clear();
                }
                else{
                    parts_list_kit_2_.clear();
                }
            }
        }
        current_order_count ++;
    }
}

bool AriacOrderManager::RemoveFailureParts(int sensor_num, std::string product_type){

    if (camera_.get_faulty_parts_num(sensor_num)>0){
        std::string product_part_frame;
        if(sensor_num==1){
            product_part_frame = parts_list_kit_1_.back();
            parts_list_kit_1_.pop_back();
        }
        else{
            product_part_frame = parts_list_kit_2_.back();
            parts_list_kit_2_.pop_back();
        }
        std::string delimiter = "_";
        unsigned int pos = 0;
        unsigned int count = 0;
        std::string token;
        int indexNum = 6;
        if (product_type == "piston_rod_part"){
            indexNum++;
        }
        while ((pos = product_part_frame.find(delimiter)) != std::string::npos) {
            token = product_part_frame.substr(0, pos);
            count++;
            product_part_frame.erase(0, pos + 1);
            if (count == indexNum){
                break;
            }
        }
        std::string product_frame = "quality_control_sensor_"+std::to_string(sensor_num)+
                                    "_model_"+token+"_frame";
        ROS_INFO_STREAM(">>>>>> Faulty part frame : "<< product_frame);

        auto part_pose = camera_.GetPartPose("/world",product_frame);


        if(product_type == "pulley_part")
            part_pose.position.z += 0.08;
        //--task the robot to pick up this part
        bool failed_pick;
        if (sensor_num==1){
            failed_pick = arm1_.PickPart(part_pose);
        }
        else {
            failed_pick = arm2_.PickPart(part_pose);
        }
        ROS_WARN_STREAM("Picking up state " << failed_pick);
        ros::Duration(1).sleep();

        while(!failed_pick){
            auto part_pose = camera_.GetPartPose("/world",product_frame);
            if (sensor_num==1){
                failed_pick = arm1_.PickPart(part_pose);
            }
            else {
                failed_pick = arm2_.PickPart(part_pose);
            }
        }

        

        bool result;
        if (sensor_num==1){
            part_pose.position.z += 0.3;
            part_pose.position.x = 0.3;
            part_pose.position.y = 2.5;
            result = arm1_.DropPart(part_pose);

        }
        else{
            part_pose.position.z += 0.3;
            part_pose.position.x = 0.3;
            part_pose.position.y = -2.5;
            result = arm2_.DropPart(part_pose);
        }


        return true;

    }
    else{
        return false;
    }

}

void AriacOrderManager::SubmitAGV(int num, std::string shipmentID) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    srv.request.shipment_type = shipmentID;
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}
