#ifndef __PLANNING_H__
#define __PLANNING_H__

#include <vector>
#include <iostream>

#include <ros/ros.h>


class Planning{

private:

    bool trigger1 = false;
    bool trigger2 = false;
    bool trigger3 = false;
    bool trigger4 = true;

    int index = 0;

public:

    Planning(){};
    ~Planning(){};
    std::vector<std::vector<double>> Select(
        std::vector<std::vector<double>>  waypointsfirst,
        std::vector<std::vector<double>>  waypointssecond,
        std::vector<std::vector<double>>  waypointsthird,
        std::vector<std::vector<double>>  waypointsforth);

};

std::vector<std::vector<double>> Planning::Select(
    std::vector<std::vector<double>>  waypointsfirst,
    std::vector<std::vector<double>>  waypointssecond,
    std::vector<std::vector<double>>  waypointsthird,
    std::vector<std::vector<double>>  waypointsforth){

    if(trigger1 == true) {

        return waypointsfirst;

    }
    else if(trigger2 == true) {

        return waypointssecond;

    }
    else if(trigger3 == true) {

        return waypointsthird;

    }
    else if(trigger4 = true){

        index++;
        std::cout<<"index : " << index << std::endl;
        //
        if(index == 300){
            trigger3 = true;
            trigger4 = false;
        }
        return waypointsforth;

    }


}
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