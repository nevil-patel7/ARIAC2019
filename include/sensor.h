//
// Created by zeid on 2/27/20.
//

#pragma once


#include <list>
#include <map>
#include <string>
#include <algorithm>
#include <vector>
#include <tuple>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

#include "ariac_part_manager.h"
// #include "conveyor_controller.h"

class AriacSensorManager {
public:
    AriacSensorManager();
    ~AriacSensorManager();
    void LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera6Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera7Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void QualitySensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void QualitySensor2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LaserProfilerCallback(const sensor_msgs::LaserScan::ConstPtr & laser_msg);

    geometry_msgs::Pose GetPartPose(const std::string& src_frame,
                                    const std::string& target_frame);
    geometry_msgs::Pose GetPartPoseFromConv(const std::string& src_frame,
                                    const std::string& target_frame);
    // std::map<std::string, std::vector<std::string>> get_product_frame_list(){
    //     return product_frame_list_;
    // }
    std::map<std::string, std::vector<std::pair<std::string, geometry_msgs::Pose>>> get_product_frame_list(){
        return products_list_;
    }
    std::map<std::string, std::vector<std::string>> get_product_frame_list_conv(){
        return product_frame_list_conv_;
    }
    std::vector<geometry_msgs::Pose> get_part_list_faulty(){
        return part_list_faulty_;
    }
    //void ScanParts(int cam_number);
    void BuildProductFrames(int);
    void breakBeam1Callback(const osrf_gear::Proximity::ConstPtr &); 
    void breakBeam2Callback(const osrf_gear::Proximity::ConstPtr &); 
    bool get_break_beam_status(int id){
        if (id ==1)
            return break_beam_1_;
        else
            return break_beam_2_;
    }

    int get_break_beam_trig_counter(int id){
        if (id ==1)
            return break_beam_1_trig_counter_;
        else
            return break_beam_2_trig_counter_;
    }

    int get_faulty_parts_num(int id){
        if (id == 1){
            return faulty_parts_1_num_;
        } else{
            return faulty_parts_2_num_;
        }
    }

    std::vector<std::tuple<std::string,double,geometry_msgs::Pose,int>> get_conveyor_list(){
        return conveyor_parts_;
    }

    // void reset();

private:
    ros::NodeHandle sensor_nh_;
    ros::Subscriber camera_1_subscriber_;
    ros::Subscriber camera_2_subscriber_;
    ros::Subscriber camera_3_subscriber_;
    ros::Subscriber camera_4_subscriber_;
    ros::Subscriber camera_5_subscriber_;
    ros::Subscriber camera_6_subscriber_;
    // ros::Subscriber camera_7_subscriber_;
    ros::Subscriber break_beam_1_subscriber_;
    // ros::Subscriber break_beam_2_subscriber_;
    ros::Subscriber quality_sensor_1_subscriber_;
    ros::Subscriber quality_sensor_2_subscriber_;
    ros::Subscriber laser_profiler_subscriber_;

    // AriacConveyorManager convBelt_Sensor_;

    tf::TransformListener camera_tf_listener_;
    tf::StampedTransform camera_tf_transform_;

    osrf_gear::LogicalCameraImage current_parts_1_;
    osrf_gear::LogicalCameraImage current_parts_2_;
    osrf_gear::LogicalCameraImage current_parts_3_;
    osrf_gear::LogicalCameraImage current_parts_4_;
    osrf_gear::LogicalCameraImage current_parts_5_;
    osrf_gear::LogicalCameraImage current_parts_6_;
    osrf_gear::LogicalCameraImage current_parts_7_;
    osrf_gear::LogicalCameraImage quality_parts_1_;
    osrf_gear::LogicalCameraImage quality_parts_2_;
    std::map<std::string, std::vector<geometry_msgs::Pose>> part_list_;

    std::map<std::string, std::list<std::string>> parts_list_;
    std::map<std::string, std::vector<std::pair<std::string, geometry_msgs::Pose>>> products_list_;
    std::pair<std::string, geometry_msgs::Pose> product_type_pose_;
    // std::map<std::string, std::vector<std::string>> product_frame_list_;
    std::map<std::string, std::vector<std::string>> product_frame_list_conv_;
    
    std::vector<geometry_msgs::Pose> part_list_faulty_;

    bool init_, cam_1_, cam_2_,cam_3_, cam_4_, cam_5_, cam_6_, cam_7_, break_beam_1_, break_beam_2_;
    int camera1_frame_counter_, camera2_frame_counter_, camera3_frame_counter_, camera4_frame_counter_, camera5_frame_counter_,
        camera6_frame_counter_, camera7_frame_counter_, break_beam_1_trig_counter_, break_beam_2_trig_counter_, faulty_parts_1_num_, faulty_parts_2_num_;

    std::vector<std::tuple<std::string,double,geometry_msgs::Pose,int>> conveyor_parts_;
    ros::Time start_time;
    ros::Time end_time;

    std::vector<int> slice_widths;
    std::vector<int> slice_heights;
    std::vector<int> slice_offsets;
    bool cur_slice_is_part;
    int count_;
};
