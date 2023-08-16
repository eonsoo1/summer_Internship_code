#include "/home/eonsoo/catkin_ws/src/planning_node/include/planning_node/plan/planning.h"
#include "/home/eonsoo/catkin_ws/src/planning_node/include/planning_node/waypoint_save/waypoint_save.h"
#include "/home/eonsoo/catkin_ws/src/planning_node/include/planning_node/VehicleMsgs/VehicleMsgs.h"
#include "/home/eonsoo/catkin_ws/src/planning_node/include/planning_node/search/search.h"

#define LOOP_HZ 100


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
    
    ros::init(argc, argv, "planning_node");
    ros::Time::init();
    ros::Rate loop_rate(LOOP_HZ);

    std::vector<std::vector<double>> waypointsfirst;
    std::vector<std::vector<double>> waypointssecond;
    std::vector<std::vector<double>> waypointsthird;
    std::vector<std::vector<double>> waypointsforth;
    std::vector<std::vector<double>> waypoints;
    
    GetWayToProjection(waypointsfirst, waypointssecond, waypointsthird, waypointsforth);

    
    Planning plan;
    VehicleMsgs msg4control;
    FindPoint targetpoint1;
    FindPoint targetpoint2;
    FindPoint targetpoint3;
    FindPoint targetpoint4;

    planning_node::CarlaEgoVehicleStatus car_stat;
    geometry_msgs::PoseStamped front_wheel_pose;
    geometry_msgs::PoseStamped rear_wheel_pose;
    
    targetpoint1.GetAllWaypoints(waypointsfirst);
    targetpoint2.GetAllWaypoints(waypointssecond);
    targetpoint3.GetAllWaypoints(waypointsthird);
    targetpoint4.GetAllWaypoints(waypointsforth);
    
    int number;

    while(ros::ok()){ 
        
        msg4control.SetValue(car_stat, front_wheel_pose, rear_wheel_pose);
        targetpoint1.FindTargetPoint(rear_wheel_pose);
        targetpoint2.FindTargetPoint(rear_wheel_pose);
        targetpoint3.FindTargetPoint(rear_wheel_pose);
        targetpoint4.FindTargetPoint(rear_wheel_pose);
        number = plan.SelectPoint();

        if(number == 1){
            std::cout << "============Lane 1st==============" << std::endl;
            plan.WayPoint(targetpoint1.FindTowardPoint(targetpoint1.target_idx(), rear_wheel_pose),
                          waypointsfirst);  
        };
        if(number == 2){
            std::cout << "============Lane 2nd==============" << std::endl;
            plan.WayPoint(targetpoint2.FindTowardPoint(targetpoint2.target_idx(), rear_wheel_pose),
                          waypointssecond);  
        };
        if(number == 3){
            std::cout << "============Lane 3rd==============" << std::endl;
            plan.WayPoint(targetpoint3.FindTowardPoint(targetpoint3.target_idx(), rear_wheel_pose),
                          waypointsthird);  
        };
        if(number == 4){
            std::cout << "============Lane 4th==============" << std::endl;
            plan.WayPoint(targetpoint4.FindTowardPoint(targetpoint4.target_idx(), rear_wheel_pose),
                          waypointsforth);  
        };


        ros::spinOnce();
        
        loop_rate.sleep();

    }
    return 0;

}

//  std::cout << "============Lane 1st==============" << std::endl;
//         targetpoint1.FindTargetPoint(rear_wheel_pose);
//         plan.WayPoint(
//             targetpoint1.target_idx(),
//             targetpoint1.FindTowardPoint(targetpoint1.target_idx(), rear_wheel_pose));
//         std::cout << "============Lane 2nd==============" << std::endl;
//         targetpoint2.FindTargetPoint(rear_wheel_pose);
//         plan.WayPoint(
//             targetpoint2.target_idx(), 
//             targetpoint2.FindTowardPoint(targetpoint2.target_idx(), rear_wheel_pose));
//         std::cout << "============Lane 3rd==============" << std::endl;
//         targetpoint3.FindTargetPoint(rear_wheel_pose);
//         plan.WayPoint(
//             targetpoint3.target_idx(), 
//             targetpoint3.FindTowardPoint(targetpoint3.target_idx(), rear_wheel_pose));
//         std::cout << "============Lane 4th==============" << std::endl;
//         targetpoint4.FindTargetPoint(rear_wheel_pose);
//         plan.WayPoint(
//             targetpoint4.target_idx(), 
//             targetpoint4.FindTowardPoint(targetpoint4.target_idx(), rear_wheel_pose));
        