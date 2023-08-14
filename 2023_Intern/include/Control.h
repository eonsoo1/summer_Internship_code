#include "ros/ros.h"
#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "waypoint_make/waypoint_msg.h"
#include "tf/tf.h"
#include "cmath"
#include <iostream>



/************purepursuit를 구현할 클래스*************/
class Pure{

private:    
  ros::NodeHandle n;
  ros::Subscriber MyInfo_sub;
  ros::Subscriber waypoint_first_sub;
  ros::Subscriber waypoint_second_sub;
  ros::Subscriber waypoint_third_sub;
  ros::Subscriber waypoint_forth_sub;

  ros::Subscriber speed_sub;
  ros::Publisher vis_pub;
  ros::Publisher controller_pub;
  ros::Publisher vehicle_vis_pub;  

  double look_ahead_distance;
  double wheel_base = 3.004645996; // 차량의 길이
  double alpha; // 목표지점과 차량의 사이각
  double current_target_x; //주기적으로 변경해야하는 값
  double current_target_y; //주기적으로 변경해야하는 값
  double current_target_z = 0.05; //주기적으로 변경해야하는 값
  double vehicle_x;
  double vehicle_y;
  double vehicle_z;
  double rel_x;
  double rel_y;
  double rel_z;
  double vehicle_speed;
  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;          
  double roll, pitch, yaw;
  bool trigger1 = true;
  bool trigger2 = true;
  const double PI = 3.141592;
  int index = 0;
  double ld = 8;

  std::vector<double> XCoordinates;
  std::vector<double> YCoordinates;
  visualization_msgs::Marker line_strip;


public :

  Pure();//기본 생성자
  ~Pure(){};//소멸자
  void PointCallback(const waypoint_make::waypoint_msg::ConstPtr& msg);
  void EgoVehicleCallback(const nav_msgs::Odometry::ConstPtr& msg); // subscriber callback 함수
  void SpeedCallback(const std_msgs::Float32::ConstPtr& msg);
  void WayPoint(); //rviz 상에 원하는 좌표를 찍을 수 있게 해주는 함수
  void VehicleCurrentPoint();
  void NextWaypoint();
  void GetRPY(); // Quaternion 값을 roll, pitch, yaw 로 변환 해주는 함수
  void TransformPoint();
  void AlphaCalculator(); // 내 차와 상대방 차의 사이 alpha 구하기
  double PureCalculator(); //pure pursuit 계산 하는 공식이 사용될 함수
  double ControlPid();
  double CalculateCTE();
  void PublishCommend(carla_msgs::CarlaEgoVehicleControl& msg);
  void printresult();
  void Do();
  
};
