#include "/home/eonsoo/catkin_ws/src/planning_node/include/planning_node/plan/planning.h"
#include "/home/eonsoo/catkin_ws/src/planning_node/include/planning_node/waypoint_save/waypoint_save.h"
#include "/home/eonsoo/catkin_ws/src/planning_node/include/planning_node/VehicleMsgs/VehicleMsgs.h"
#include "vector"

#define LOOP_HZ 10


void GetWayToProjection(

    std::vector<std::vector<double>>& first,
    std::vector<std::vector<double>>& second,
    std::vector<std::vector<double>>& third,
    std::vector<std::vector<double>>& forth){
        
    GetWaypoints(first, "/home/eonsoo/Map/lane1st.csv");
    GetWaypoints(second, "/home/eonsoo/Map/lane2nd.csv");
    GetWaypoints(third, "/home/eonsoo/Map/lane3rd.csv");
    GetWaypoints(forth, "/home/eonsoo/Map/lane4th.csv");
    
    // utm projection

    LatLon2Utm(first);
    LatLon2Utm(second);
    LatLon2Utm(third);
    LatLon2Utm(forth);

}

int main(int argc, char** argv) {
    
    std::vector<std::vector<double>> waypointsfirst;
    std::vector<std::vector<double>> waypointssecond;
    std::vector<std::vector<double>> waypointsthird;
    std::vector<std::vector<double>> waypointsforth;
    std::vector<std::vector<double>> waypoints;
    
    GetWayToProjection(
        waypointsfirst,
        waypointssecond, 
        waypointsthird,
        waypointsforth);


    ros::init(argc, argv, "planning_node");
    ros::Time::init();
    ros::Rate loop_rate(LOOP_HZ);

    VehicleMsgs msg4control;
    Planning plan;
    planning_node::CarlaEgoVehicleStatus car_stat;
    geometry_msgs::PoseStamped front_wheel_pose;
    geometry_msgs::PoseStamped rear_wheel_pose;
    
    while(ros::ok()){

      msg4control.SetValue(car_stat, front_wheel_pose, rear_wheel_pose);
      waypoints = plan.SelectPoint(waypointsfirst, waypointssecond, waypointsthird, waypointsforth);
      plan.WayPoint(waypoints);
      ros::spinOnce();
      loop_rate.sleep();   
    }
    return 0;

}

