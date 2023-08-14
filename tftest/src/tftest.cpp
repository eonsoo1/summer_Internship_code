#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "waypoint_make/waypoint_msg.h" 
#include <iostream>

#define set_index 100

class Local{

private:
    ros::Subscriber path_sub;
    ros::Subscriber my_location_sub;
    ros::NodeHandle nh;
    std::vector<double> XCoordinates;
    std::vector<double> YCoordinates;
    
    double rel_x;
    double rel_y;
    double vehicle_x;
    double vehicle_y;
    double vehicle_z;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;
    double roll, pitch, yaw;
    
    double target_distance;
    double alpha;

    bool trigger = true;
    int temp_index = 0;
    int count;

public:
    Local(){
        my_location_sub = nh.subscribe("/carla/ego_vehicle/odometry", 100, &Local::EgoVehicleCallback, this);
        path_sub = nh.subscribe("point_msgs", 100, &Local::pathCallback, this);
    }
    ~Local(){};

    void pathCallback(const waypoint_make::waypoint_msg::ConstPtr& std_msgs);
    void EgoVehicleCallback(const nav_msgs::Odometry::ConstPtr& msg);
    

    void SetRange(int index);
    void FindClosestPoint(std::vector<double> waypoint_x, std::vector<double> waypoint_y, int init_index, int end_index);
    void TransformPoint(double target_x, double target_y);
    void GetRPY();

    void Do();

};


void Local::pathCallback(const waypoint_make::waypoint_msg::ConstPtr& msg){
    if(trigger == true){
        for(int i = 0; i < msg->points.size() ; i++ ){
        XCoordinates.push_back(msg -> points[i].x);//[points vector size 로 for 문 돌리기]
        YCoordinates.push_back(msg -> points[i].y);  
        std::cout<< "x = " << XCoordinates[i] << "    ";
        std::cout<< "y = " << YCoordinates[i] << std::endl;
        } 
        trigger = false;
    }

}

void Local::EgoVehicleCallback(const nav_msgs::Odometry::ConstPtr& msg){
    
    vehicle_x = msg-> pose.pose.position.x;
    vehicle_y = msg-> pose.pose.position.y;
    vehicle_z = msg-> pose.pose.position.z;
    orientation_x = msg-> pose.pose.orientation.x;
    orientation_y = msg-> pose.pose.orientation.y;
    orientation_z = msg-> pose.pose.orientation.z;
    orientation_w = msg-> pose.pose.orientation.w;

}

// /*************현재 바라보고 있는 target point visualiazation**************/
// void Local::WayPoint(){

//   visualization_msgs::Marker marker;

// 	marker.header.frame_id = "map";

// 	marker.header.stamp = ros::Time();

//   /**********************마커의 네임스페이스************************/
// 	marker.ns = "my_namespace";

//   /********************* 마커에 할당된 고유 ID**********************/
// 	marker.id = 0;

//   /*********The available types are specified in the message definition.********/
// 	marker.type = visualization_msgs::Marker::CUBE;

//   /**********0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall***********/
// 	marker.action = visualization_msgs::Marker::ADD;

//   /****************Pose marker, specified as x/y/z position ***************/
// 	marker.pose.position.x = current_target_x;
// 	marker.pose.position.y = current_target_y;
// 	marker.pose.position.z = current_target_z;

//   /******************* x/y/z/w quaternion orientation.*********************/
// 	marker.pose.orientation.x = 0.0;
// 	marker.pose.orientation.y = 0.0;
// 	marker.pose.orientation.z = 0.0;
// 	marker.pose.orientation.w = 1.0; //쿼터니언 값(?)
	
//   /******************마커의 scale [1,1,1] = 1m x 1m x 1m******************/
//   marker.scale.x = 1;
// 	marker.scale.y = 1;
// 	marker.scale.z = 1;

//   /***************************객체의 색상*****************************/
// 	marker.color.a = 1.0; // Don't forget to set the alpha!
// 	marker.color.r = 1.0;
// 	marker.color.g = 0.0;
// 	marker.color.b = 0.0;

// 	//only if using a MESH_RESOURCE marker type:
// 	// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  
// 	vis_pub.publish( marker );

// }

/**********target_x, y를 차량에 대한 상대좌표로 바꾸어 주는 함수 **********/

void Local::TransformPoint(double target_x, double target_y){

    
    alpha = atan2((target_y - vehicle_y), (target_x - vehicle_z)) - yaw;
    target_distance = sqrt(pow((target_x - vehicle_x), 2) + pow((target_y - vehicle_y), 2));
    
    rel_x = target_distance * cos(alpha);
    rel_y = target_distance * sin(alpha);

}

void Local::GetRPY(){

  tf::Quaternion q(orientation_x, orientation_y, orientation_z, orientation_w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

}

/********차량으로부터 가장 가까운 점 찾는 함수**********/
// waypoint_x, y 는 path의 x, y vector를 넣는다.
// 전체 경로를 매 번 반복해서 찾으면 시간이 오래걸릴 것 같으니, 특정 범위 구간을 정하여 넣어주면 좋을 듯 싶다.
void Local::FindClosestPoint(std::vector<double> waypoint_x, std::vector<double> waypoint_y, int init_index, int end_index){
    
    double min_distance;
    double prev_distance = sqrt(pow((waypoint_x[0] - vehicle_x), 2) + pow((waypoint_y[0] - vehicle_y), 2));;
    temp_index = init_index;
    
    for(int i = init_index ; i < end_index ; i++){

        TransformPoint(waypoint_x[i], waypoint_y[i]); 
        min_distance = sqrt(pow((rel_x), 2) + pow((rel_y), 2));
        
        if(min_distance < prev_distance){
            
            prev_distance = min_distance;
            temp_index = i;
            
        }
    }
    
    TransformPoint(waypoint_x[temp_index], waypoint_y[temp_index]);
    std::cout << "The closest point : (" << rel_x << ", " << rel_y << ")" << std::endl;

    // 내 차량에서 가장 가까운 점까지 위에서 구함
    // 아래 부터는 가까운 점부터 정헤둔 지점까지 baseframe을 만든다.

    for(int i = temp_index ; i < set_index ; i++){
    
        TransformPoint(waypoint_x[i], waypoint_y[i]);
        rel_x;
        rel_y;    
        
    }

}

/***********가장가까운 점을 찾을 범위를 지정해주는 함수************/
void Local::SetRange(int index){

    FindClosestPoint(XCoordinates, YCoordinates, index, index + 20);

}

void Local::Do(){

    std::cout << "=======================================" << std::endl;
    std::cout << " vehicle x = " << vehicle_x << std::endl;
    std::cout << " vehicle y = " << vehicle_y << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    GetRPY();
    SetRange(temp_index);

}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "Local_Planning");
    Local local;
    ros::Rate loop_rate(100);
    while(ros::ok()){
        
        local.Do();
        ros::spinOnce();
        loop_rate.sleep();
    
    }

}