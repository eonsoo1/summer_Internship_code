// #include "ros/ros.h"
// #include "carla_msgs/CarlaEgoVehicleControl.h"
// #include "carla_msgs/CarlaEgoVehicleStatus.h"
// #include "geometry_msgs/Quaternion.h"
// #include "geometry_msgs/Point.h"
// #include <geometry_msgs/PointStamped.h>
// #include <geometry_msgs/PoseStamped.h>
// #include "nav_msgs/Odometry.h"
// #include "std_msgs/Float32.h"
// #include "visualization_msgs/Marker.h"
// #include "visualization_msgs/MarkerArray.h"
// #include "waypoint_make/waypoint_msg.h"
// #include "tf/tf.h"
// #include <tf/transform_listener.h>
// #include <tf/transform_broadcaster.h>
#include "vector"
#include "cmath"
#include <iostream>

#define point_number 50
#define offset -10
class lattice{

private:
    std::vector<double> point_x;
    std::vector<double> point_y;
    double center_point_x;
    double center_point_y;
    double center_point_tangent;
    double vertical_tan;
    double side_x;
    double side_y;
public:
    lattice(){};
    ~lattice(){};
    void makePoint();
    void findCenterPoint();
    void findSidePoint();
    void Do();
};