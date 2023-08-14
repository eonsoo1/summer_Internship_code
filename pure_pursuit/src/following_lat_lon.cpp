/*
 * todo list
 * 1. control by gnss data
 * 2. calculate heading by....wha??
 */


#include "pure_pursuit/waypoint_save/waypoint_save.h"
#include "pure_pursuit/ControlMsgs/ControlMsgs.h"
#include "pure_pursuit/PurePursuitControl.h"
#include "pure_pursuit/pid/pid.h"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_projection/UTM.h"


#define P_GAIN          3.
#define I_GAIN          0.
#define D_GAIN          1.

#define TARGET_VEL_MS   10.


int main(int argc, char** argv) {

    ros::init (argc, argv, "following_lat_lon");
    ros::Time::init();
    ros::Rate loop_rate_hz(60.0);

    // bagfile car odometry -> waypoints
    std::vector<std::vector<double>> waypointsfirst;
    std::vector<std::vector<double>> waypointssecond;
    std::vector<std::vector<double>> waypointsthird;
    std::vector<std::vector<double>> waypointsforth;

    GetWaypointsTwo(waypointsfirst, "/home/eonsoo/Map/lane1st.csv");
    GetWaypointsTwo(waypointssecond, "/home/eonsoo/Map/lane2nd.csv");
    GetWaypointsTwo(waypointsthird, "/home/eonsoo/Map/lane3rd.csv");
    GetWaypointsTwo(waypointsforth, "/home/eonsoo/Map/lane4th.csv");
    
    // utm projection

    ProjectorCoords(waypointsfirst);
    ProjectorCoords(waypointssecond);
    ProjectorCoords(waypointsthird);
    ProjectorCoords(waypointsforth);
 

    // subscribe and publise
    ControlMsgs msgs;

    PID longitudinal_pid = PID((1./60.), 1., 0., P_GAIN, D_GAIN, I_GAIN);
    double throttle_pid;

    // pure pursuit calculation
    PurePursuitControl pp_ctr;
    int target_wypt_idx = 0;
    double steer_output;

    while(ros::ok()) {

        throttle_pid = longitudinal_pid.calculate(TARGET_VEL_MS, msgs.vehicle_status.velocity);

        steer_output = pp_ctr.SetSteer(
            waypointsfirst[target_wypt_idx][POSITION_X],
            waypointsfirst[target_wypt_idx][POSITION_Y]
        );


        std::cout << 
            "target waypoint index : " << target_wypt_idx << "\n" <<
            "waypoint x : " << waypointsfirst[target_wypt_idx][POSITION_X] << "\n" <<
            "waypoint y : " << waypointsfirst[target_wypt_idx][POSITION_Y] << "\n" <<
            "steer : " << steer_output << "\n" << 
            "==================================================\n" <<
        std::endl;

        msgs.PubMsg(throttle_pid, steer_output, 0.);

        if (target_wypt_idx == waypointsfirst.size()-1) {
            return 0;
        }

        if (pp_ctr.ArrivedLKDist() == true) {
            target_wypt_idx++;
        }
        
        pp_ctr.print_val();

        ros::spinOnce();
        loop_rate_hz.sleep();
    }


    return 0;
}