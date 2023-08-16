#ifndef __PLANNING_H__
#define __PLANNING_H__

#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"

#include <ros/ros.h>
#include "planning_node/waypoint_msg.h"
#include "/home/eonsoo/catkin_ws/src/planning_node/include/planning_node/waypoint_save/waypoint_save.h"
#include "detection_traffic_sign/red.h"

class Planning{

private:
    ros::Publisher point_pub;
    ros::Subscriber trafficsign_sub;
    ros::NodeHandle n;
    planning_node::waypoint_msg msg;
 

    bool trigger1 = false;
    bool trigger2 = false;
    bool trigger3 = false;
    bool trigger4 = true;
    bool traffic;

    int index = 0;

public:

    Planning(){
        point_pub = n.advertise<planning_node::waypoint_msg>("point_msgs", 1000);
        trafficsign_sub = n.subscribe("stop", 1, &Planning::SignCallback, this);
    };
    ~Planning(){};
    void PubInfo(int toward_idx, std::vector<std::vector<double>> waypoints, bool trigger);
 
    void SignCallback(const detection_traffic_sign::red::ConstPtr& msg_sign);
    int SelectPoint();

};
void Planning::SignCallback(const detection_traffic_sign::red::ConstPtr& msg_sign){

    traffic = msg_sign->red;
    std::cout<< "traffic_sign : " << traffic << std::endl;
}

int Planning::SelectPoint(){

    index++;
    std::cout<<"index : " << index << std::endl;
    
    if(trigger1 == true) {

        if(index == 400){
            trigger1 = false;
            trigger4 = true;
        }
        return 1;

    }
    else if(trigger2 == true) {

        if(index == 300){
            trigger2 = false;
            trigger1 = true;
        }
        return 2;

    }
    else if(trigger3 == true) {

        
        if(index == 200){
            trigger3 = false;
            trigger2 = true;
        }

        return 3;

    }
    else if(trigger4 = true){

        if(index == 100){
            trigger3 = true;
            trigger4 = false;
        }

        return 4;

    }


}


void Planning::PubInfo(int toward_idx, std::vector<std::vector<double>> waypoints, bool trigger){
    
  /**********Control node로 보내기 위한 변수***********/
  
    std::cout <<"toward_index = " << toward_idx << std::endl; 
    std::cout << "x = " << waypoints[toward_idx][0] << std::endl; 
    std::cout << "y = " << waypoints[toward_idx][1] << std::endl; 
    std::cout << "overtaking :  " << trigger << std::endl; 
    msg.point.x = waypoints[toward_idx][0];
    msg.point.y = waypoints[toward_idx][1];
    msg.toward_target_index = toward_idx;
    msg.overtake_trigger = trigger;
    point_pub.publish(msg);

}




// void Planning::PubInfo(std::vector<std::vector<double>>& waypoints){


//   /**********Control node로 보내기 위한 변수***********/
//     planning_node::waypoint_msg msg;
    
//   /**********Control node로 보내기 위한 식***********/
//     for (size_t i = 0; i < waypoints.size(); ++i){

//       geometry_msgs::Point point;
//       point.x = waypoints[i][POSITION_X];
//       point.y = waypoints[i][POSITION_Y];
//       msg.points.push_back(point);
      
//     //   std::cout << "x["<< i<<"] : " << point.x << std::endl;
//     //   std::cout << "y : " << point.y << std::endl;
//     }
    
//     point_pub.publish(msg);

// }




// frenet 좌표계로 변환

class FrenetConverter {
public:

    FrenetConverter(const std::vector<std::vector<double>>& reference_trajectory) // 복사생성자 : 2차원 벡터 하나를 받아와서 레퍼런스 경로를 선언 & 정의.
        : reference_trajectory_(reference_trajectory) {};


    // cartesian 벡터를 받아와서 frenet_points라고 하는 벡터에 convertSinglePointToSDFrenet함수를 통해 배열별로 변환 후 pushback한다.
    // frenet_points를 리턴한다.
    std::vector<std::vector<double>> convertToSDFrenet(const std::vector<std::vector<double>>& cartesian_points) { 

        std::vector<std::vector<double>> frenet_points;

        for (const auto& cartesian_point : cartesian_points) {
            std::vector<double> frenet_point = convertSinglePointToSDFrenet(cartesian_point);
            frenet_points.push_back(frenet_point);
        }
    
        return frenet_points;
    }

private:
    std::vector<std::vector<double>> reference_trajectory_;

    // 클래스 내부 함수에서만 호출할 수 있도록 private으로 선언한듯하다.
    std::vector<double> convertSinglePointToSDFrenet(const std::vector<double>& cartesian_point) {
    // Assume that reference_trajectory_ contains the reference points for Frenet calculation
    const std::vector<std::vector<double>>& reference_trajectory = reference_trajectory_;

    double min_distance = std::numeric_limits<double>::max();
    double min_s = 0.0;
    double min_d = 0.0;

    for (const auto& reference_point : reference_trajectory) {
        double dx = cartesian_point[0] - reference_point[0]; // Difference in x
        double dy = cartesian_point[1] - reference_point[1]; // Difference in y
        double distance = std::sqrt(dx * dx + dy * dy); // Euclidean distance

        if (distance < min_distance) {
            min_distance = distance;
            min_s = reference_point[0]; // Assume reference_point[0] corresponds to s in Frenet
            min_d = reference_point[1]; // Assume reference_point[1] corresponds to d in Frenet
        }
    }

    std::vector<double> frenet_point = {min_s, min_d};

    return frenet_point;
}
};
// int main() {
//     std::vector<CartesianPoint> reference_trajectory = {
//         CartesianPoint(0.0, 0.0),
//         CartesianPoint(1.0, 1.0),
//         CartesianPoint(2.0, 2.0),
//         CartesianPoint(3.0, 3.0)
//     };

//     CartesianPoint cartesian_point(1.5, 1.0);

//     FrenetConverter converter(reference_trajectory);
//     FrenetPoint frenet_point = converter.convertToSDFrenet(cartesian_point);

//     std::cout << "Frenet Coordinates:" << std::endl;
//     std::cout << "s: " << frenet_point.s << ", d: " << frenet_point.d << std::endl;

//     return 0;
// }
#endif //__PLANNING_H__