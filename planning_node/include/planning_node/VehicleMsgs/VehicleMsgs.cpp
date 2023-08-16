#include "/home/eonsoo/catkin_ws/src/planning_node/include/planning_node/VehicleMsgs/VehicleMsgs.h"

VehicleMsgs::VehicleMsgs() {

    fr_pose_received = false;
    rr_pose_received = false;

    vehicle_status_sub = nh.subscribe ("/carla/ego_vehicle/vehicle_status", 100, &VehicleMsgs::GetCarStatus, this);
    front_wheel_sub = nh.subscribe("/front_wheel_pose", 100, &VehicleMsgs::GetFrontWheelPose, this);
    rear_wheel_sub = nh.subscribe("/rear_wheel_pose", 100, &VehicleMsgs::GetRearWheelPose, this);


    target_vis_pub = nh.advertise<visualization_msgs::Marker>("/target_waypoint", 100);
    target_wypt_marker.header.frame_id = "map";
    target_wypt_marker.ns = "based_on_fr";
    target_wypt_marker.id = 0;
    target_wypt_marker.type = visualization_msgs::Marker::CUBE;
    target_wypt_marker.action = visualization_msgs::Marker::ADD;

    target_wypt_marker.scale.x = CUBE_SIZE;
    target_wypt_marker.scale.y = CUBE_SIZE;
    target_wypt_marker.scale.z = CUBE_SIZE;

    // orange
    target_wypt_marker.color.r = CUBE_COLOR_R;
    target_wypt_marker.color.g = CUBE_COLOR_G;
    target_wypt_marker.color.b = CUBE_COLOR_B;
    target_wypt_marker.color.a = CUBE_COLOR_A;

    target_wypt_marker.lifetime = ros::Duration();
}
 
bool VehicleMsgs::FrPoseReceived() {
    if (fr_pose_received == true) {
        return true;
    }
    else {
        return false;
    }
}


void VehicleMsgs::GetCarStatus(const planning_node::CarlaEgoVehicleStatus::ConstPtr& msg) {
    
    vehicle_status.velocity = msg->velocity;
    vehicle_status.acceleration = msg->acceleration;
    vehicle_status.orientation = msg->orientation;
    vehicle_status.control = msg->control;
}


void VehicleMsgs::GetFrontWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    fr_pose.pose = msg->pose;
    fr_pose_received = true;
}
void VehicleMsgs::GetRearWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    rr_pose.pose = msg->pose;
    rr_pose_received = true;
}   

void VehicleMsgs::SetValue(planning_node::CarlaEgoVehicleStatus& _car_stat, geometry_msgs::PoseStamped& _fr_pose, geometry_msgs::PoseStamped& _rr_pose) {
    _car_stat = vehicle_status;
    _fr_pose = fr_pose;
    _rr_pose = rr_pose;
}


void VehicleMsgs::PubVisMsg(std::vector<double> wypt) {
    target_wypt_marker.header.stamp = ros::Time::now();
    target_wypt_marker.pose.position.x = wypt[0];  // utm projected x
    target_wypt_marker.pose.position.y = wypt[1];  // utm projected y
    target_wypt_marker.pose.position.z = 0.;       
    target_wypt_marker.pose.orientation.x = 0.;
    target_wypt_marker.pose.orientation.y = 0.;
    target_wypt_marker.pose.orientation.z = 0.;
    target_wypt_marker.pose.orientation.w = 0.;

    target_vis_pub.publish(target_wypt_marker);
}