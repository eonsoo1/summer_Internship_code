#include "control_node/ControlMsgs/ControlMsgs.h"
#include "control_node/waypoint_save/waypoint_save.h"
#include "control_node/pid/pid.h"
#include "control_node/stanley/stanley.h"
#include "control_node/error_calculate/error_calculate.h"
#include "control_node/pure_pursuit/PurePursuitControl.h"
#include "control_node/Planning/planning.h"

#define LOOP_HZ                 60.
#define P_GAIN                  3.
#define I_GAIN                  0.
#define D_GAIN                  1.
#define TARGET_VELOCITY_MS      20.

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
    ros::init(argc, argv, "control_node");
    ros::Time::init();
    ros::Rate loop_rate(LOOP_HZ);


     // bagfile car odometry -> waypoints
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


    ControlMsgs msg4control;
    control_node::CarlaEgoVehicleStatus car_stat;
    geometry_msgs::PoseStamped front_wheel_pose;
    geometry_msgs::PoseStamped rear_wheel_pose;

    FollowingError error_check;

    PurePursuitControl pp;
    StanleyControl stanley;
    PID longi_control = PID((1./LOOP_HZ), 1., 0., P_GAIN, D_GAIN, I_GAIN);
    double throttle;
    double steer;

    Planning plan;

    
    bool trigger = true;
  
    int visualizing_target_idx;
   
    while (ros::ok()) {

        waypoints = plan.Select(waypointsfirst, waypointssecond, waypointsthird, waypointsforth);
    
        stanley.GetAllWaypoints(waypoints);
        pp.GetAllWaypoints(waypoints);
        
        if (visualizing_target_idx == waypoints.size()-1) {
            std::cout << error_check.err_avg() << std::endl;
            return 0;
        }
        if (msg4control.FrPoseReceived() == false) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        msg4control.SetValue(car_stat, front_wheel_pose, rear_wheel_pose);


        throttle = longi_control.calculate(TARGET_VELOCITY_MS, car_stat.velocity);


        stanley.SetSteer(front_wheel_pose, car_stat.velocity);
        pp.SetSteer(rear_wheel_pose);

        if (car_stat.velocity < 5.) {      // if car is slower than 15m/s
            
            std::cout << "using stanley controller\n";
            steer = stanley.steer_val();
            visualizing_target_idx = stanley.target_idx();
            msg4control.PubVisMsg(waypoints[visualizing_target_idx]);
        }
        else {                              // if car is faster..
            
            std::cout << "using pure pursuit controller\n";
            steer = pp.steer_val();
            visualizing_target_idx = pp.target_idx();
            msg4control.PubVisMsg(waypoints[visualizing_target_idx]);
        }
    

        msg4control.PubControlMsg(throttle, steer, 0.);

        // stanley.PrintValue();
        pp.PrintValue();

        
        // use distance from stanley
        error_check.FilteredValue(stanley.distance_m());
        error_check.PubCrossTrackError();           // publish cross track err

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}