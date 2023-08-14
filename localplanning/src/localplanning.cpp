#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h> // it needs to define Point
#include <tf/tf.h> 
#include "/home/eonsoo/catkin_ws/src/localplanning/include/localplanning/localplanning.h"


void LocalPlanning::waypointCallback(const waypoint_make::waypoint_msg::ConstPtr& msg){
    if(trigger == true){
        for(int i = 0; i < msg->points.size() ; i++ ){   
            XCoordinates.push_back(msg -> points[i].x);//[points vector size 로 for 문 돌리기]
            YCoordinates.push_back(msg -> points[i].y);  
            // std::cout<< "x = " << XCoordinates[i] << "    ";
            // std::cout<< "y = " << YCoordinates[i] << std::endl;
        }
        trigger = false;
    }
}

void LocalPlanning::EgoVehicleCallback(const nav_msgs::Odometry::ConstPtr& msg){
    
    vehicle_x = msg-> pose.pose.position.x;
    vehicle_y = msg-> pose.pose.position.y;
    vehicle_z = msg-> pose.pose.position.z;
    orientation_x = msg-> pose.pose.orientation.x;
    orientation_y = msg-> pose.pose.orientation.y;
    orientation_z = msg-> pose.pose.orientation.z;
    orientation_w = msg-> pose.pose.orientation.w;
    
}

void LocalPlanning::GetRPY(){

    tf::Quaternion q(orientation_x, orientation_y, orientation_z, orientation_w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    
}

void LocalPlanning::makeWaypoint(){

    for(int i = 0 ; i < 100 ; i++){
        XCoordinates.push_back(i);
        YCoordinates.push_back(i);
    }
}

void LocalPlanning::printresult(){
    std::cout<< "================================" << std::endl;
    std::cout<< "vehicle x = " << vehicle_x << std::endl;
    std::cout<< "vehicle y = " << vehicle_y << std::endl;
    std::cout<< "vehicle z = " << vehicle_z << std::endl;
    std::cout<< "vehicle yaw = " << yaw << std::endl;
    std::cout<< " " << std::endl;
}

void LocalPlanning::Do(){

    GetRPY();
    printresult();

}


int main(int argc, char** argv){

	ros::init(argc, argv, "transform_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
	LocalPlanning local;
	while(ros::ok()){
        
        local.Do();
	    ros::spinOnce();

    }
    loop_rate.sleep();

}