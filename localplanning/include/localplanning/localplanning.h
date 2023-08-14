#include "ros/ros.h"
#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "waypoint_make/waypoint_msg.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "cmath"
#include <iostream>

class LocalPlanning{
private :

    ros::Subscriber waypoint_sub;
    ros::Subscriber front_wheel_sub;
    ros::Subscriber MyInfo_sub;
    //ros::Timer timer;
    ros::NodeHandle n;
    
    geometry_msgs::PoseStamped fr_pose;
    std::vector<double> XCoordinates;
    std::vector<double> YCoordinates;
    double vehicle_x;
    double vehicle_y;
    double vehicle_z;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;
    double roll, pitch, yaw;
    double current_target_x; //주기적으로 변경해야하는 값
    double current_target_y; //주기적으로 변경해야하는 값
    double current_target_z = 0.05; //주기적으로 변경해야하는 값
    bool msg_received;
    bool trigger = true;

public:
    LocalPlanning(){
        waypoint_sub = n.subscribe("point_msgs", 100, &LocalPlanning::waypointCallback, this);
        //front_wheel_sub = n.subscribe("/front_wheel_pose", 100, &LocalPlanning::GetFrontWheelPose, this);
        MyInfo_sub = n.subscribe("/carla/ego_vehicle/odometry", 100, &LocalPlanning::EgoVehicleCallback, this);
    
        // timer = n.createTimer(ros::Duration(1.0), boost::bind(&LocalPlanning::transformPoint, boost::ref(listener)), this);
    };
    ~LocalPlanning(){};
    
    void waypointCallback(const waypoint_make::waypoint_msg::ConstPtr& msg);
    void EgoVehicleCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void GetRPY();
    void transformPoint();
    void GetFrontWheelPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void convertCoordinate(std::vector<tf::Vector3> &map_coordinates);
    void makeWaypoint();
    void printresult();
    void Do();
};

