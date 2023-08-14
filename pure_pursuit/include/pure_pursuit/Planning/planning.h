// #ifndef __PLANNING_H__
// #define __PLANNING_H__

#include <vector>
#include <iostream>

#include <ros/ros.h>

class Planning{

private:
    bool trigger1 = false;
    bool trigger2 = false;
    bool trigger3 = false;
    bool trigger4 = false;

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
    if(trigger2 == true) {

        return waypointssecond;

    }
    if(trigger3 == true) {

        return waypointsthird;

    }
    else{

        index++;
        std::cout<<"index : " << index << std::endl;
        if(index == 300){
            trigger2 = true;
        }
        return waypointsforth;

    }


}
// #endif //__PLANNING_H__