#ifndef __PLANNING_NODE_H__
#define __PLANNING_NODE_H__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include "planning_node/CarlaEgoVehicleStatus.h"
#include "planning_node/CarlaEgoVehicleControl.h"


#define CUBE_SIZE           0.5
#define CUBE_COLOR_R        1.0f
#define CUBE_COLOR_G        0.8f
#define CUBE_COLOR_B        0.5f
#define CUBE_COLOR_A        1.0f    // orange


class VehicleMsgs {

    ros::NodeHandle nh;
    
    ros::Subscriber vehicle_status_sub;
    planning_node::CarlaEgoVehicleStatus vehicle_status;

    ros::Subscriber front_wheel_sub;
    ros::Subscriber rear_wheel_sub;
    geometry_msgs::PoseStamped fr_pose;
    geometry_msgs::PoseStamped rr_pose;
    bool fr_pose_received;
    bool rr_pose_received;


    planning_node::CarlaEgoVehicleControl ctrl_pub_msg;

    ros::Publisher target_vis_pub;
    visualization_msgs::Marker target_wypt_marker;

public:

    VehicleMsgs ();
    bool FrPoseReceived();
    
    void GetCarStatus(const planning_node::CarlaEgoVehicleStatus::ConstPtr& msg);
    void GetFrontWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void GetRearWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void SetValue(planning_node::CarlaEgoVehicleStatus& _car_stat, geometry_msgs::PoseStamped& _fr_pose, geometry_msgs::PoseStamped& _rr_pose);
    
    void PubVisMsg(std::vector<double> wypt);


};






#endif // __PLANNING_NODE_H__