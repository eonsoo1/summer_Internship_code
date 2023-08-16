#ifndef __STANLEY__
#define __STANLEY__

#include <ros/ros.h>
#include <tf/tf.h>


#define SEARCH_IDX_RANGE            100
#define ARCTAN_TERM_CONSTANT        5.
#define ARCTAN_TERM_MIN_VELOCITY    0.01
#define MAX_STEERING_DEG            34.99999642
#define MIN_STEERING_DEG            -34.99999642
#define LOOKAHEAD_DIST              40.


class FindPoint {

    std::vector<std::vector<double>> waypoints;
    int target_search_idx;
    int target_wypt_idx;
    tf::Transform front_wheel_tf;
    tf::Transform target_wypt_tf;
    tf::Transform next_wypt_tf;
    double car_wypt_dist_m;
    double compare_dist_m;

    double psi_deg;
    double arctan_term_deg;
    double steering_val;    // -1.0 ~ 1.0 value


public:

    FindPoint();
    void GetAllWaypoints(std::vector<std::vector<double>> wypts);

    // set waypoint, its distance, and its yaw
    void FindTargetPoint(geometry_msgs::PoseStamped fr_pose);
//     void GetPsi();
//     void GetArcTanTerm(double _velocity);
//     double SetSteer(geometry_msgs::PoseStamped _fr_pose, double velocity);

    int FindTowardPoint(int target_idx, geometry_msgs::PoseStamped fr_pose);

    int target_idx();
//     double distance_m();
//     double steer_val();
//     void PrintValue();    
// 
};


#endif // __STANLEY__