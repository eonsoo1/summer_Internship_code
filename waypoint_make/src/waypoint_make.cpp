#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "waypoint_make/waypoint_msg.h"

/******Waypoint txt 파일을 x, y 좌표값으로 바꾸기 위한 클래스******/

struct Point{
  
  double x;
  double y;

};

class MyPoint{
  
  std::vector<double> waypoint;
  std::vector<double> xCoordinates;
  std::vector<double> yCoordinates;
  ros::Publisher vis_pub;
  ros::Publisher point_pub;
  ros::NodeHandle n;
  
public:

   MyPoint(){
    
    Point point;
    ReadFile();
    vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 1000 );
    point_pub = n.advertise<waypoint_make::waypoint_msg>("point_msgs", 1000);

   };
  ~MyPoint(){};
  void PushBack(visualization_msgs::MarkerArray& s); // for문을 돌면서 만든 marker를 markerarray로 push_back하는 함수
  void MakeArray(std::vector<double> vector_input); // 멤버변수로 선언한 벡터에 값을 넣어주는 함수
  void ReadFile(); // waypoint.txt 파일에서 값을 읽어와 배열로 만들어주는 함수 
  void PipeLine(); // 함수 호출하는 함수
  void WayPoint();
    /*************vector 메모리 제거**************/
  void Clear(){
    std::vector<double>().swap(xCoordinates);
    std::vector<double>().swap(yCoordinates);
  };

  std::vector<std::string> Split(std::string input, char point);  //ReadFile에서 받은 파일 값의 한 줄을 ','를 기준으로 나누는 함수*
};

/************ ReadFile에서 받은 파일 값의 한 줄을 ','를 기준으로 나누는 함수***************/
std::vector<std::string> MyPoint::Split(std::string input, char point){
    
    std::vector<std::string> oneline;
    std::istringstream ss(input);
    std::string output;

    while(getline(ss, output, point)){
        
        oneline.push_back(output);

    }
    return oneline;
}

/********waypoint.txt 파일에서 값을 읽어와 double 타입으로 만들어주는 함수*********/
void MyPoint::ReadFile(){

    std::vector<std::string> waypoint;
    std::vector<std::string> waypoint_temp;
    std::vector<double> waypoint_double;
    // 파일 읽기 준비
    std::ifstream in("/home/eonsoo/catkin_ws/src/waypoint_make/src/Points.txt");

    if (!in.is_open()) {
        std::cout << "파일을 찾을 수 없습니다!" << std::endl;
        return;
    }

    while (in) {
        
        std::string s;
        getline(in, s);
        waypoint = Split(s, ',');
        std::vector<std::string>::iterator iter;
        for(iter = waypoint.begin(); iter!= waypoint.end(); iter++){
            waypoint_temp.push_back(*iter);
        }

    //split이라는 함수를 만들어서 한 줄 받으면 ','으로 나누어서 그 함수의 지역 변수에 저장
    }
    std::vector<std::string>::iterator iter_once;
    for(iter_once = waypoint_temp.begin(); iter_once!= waypoint_temp.end(); iter_once++){
        std::istringstream ss(*iter_once);
        double x;
        ss >> x;
        waypoint_double.push_back(x);
    }

  MakeArray(waypoint_double);
  
}

/************ x, y 벡터를 만들어주는 함수*************/
void MyPoint::MakeArray(std::vector<double> vector_input){ 
    
    int count = 0;
    std::vector<double>::iterator iter;
    for(iter = vector_input.begin(); iter!= vector_input.end(); iter++){
       if(count%2 == 0){
       xCoordinates.push_back(*iter);
       }
       else{
       yCoordinates.push_back(*iter);
       }      
       count++;
      
    }
    
    // for(int i = 0 ; i < xCoordinates.size() ; i++){
    //    ROS_INFO("x = %f", xCoordinates[i]);
    //    ROS_INFO("y = %f", yCoordinates[i]); 
    // }
    // std::vector<double>::iterator iter_x;
    // for(iter_x = xCoordinates.begin(); iter_x!= xCoordinates.end(); iter_x++){
    //   std::cout << "Iterator x " << *iter_x << std::endl; 
    // }
    // std::vector<double>::iterator iter_y;
    // for(iter_y = yCoordinates.begin(); iter_y!= yCoordinates.end(); iter_y++){
    //    std::cout << "Iterator y " << *iter_y << std::endl; 
    // }

}

void MyPoint::WayPoint(){

  /**********Control node로 보내기 위한 변수***********/
    waypoint_make::waypoint_msg msg;
    
   

  /**********Control node로 보내기 위한 식***********/
    for (size_t i = 0; i < xCoordinates.size(); ++i){

      geometry_msgs::Point point;
      point.x = xCoordinates[i];
      point.y = yCoordinates[i];
      msg.points.push_back(point);
      
      std::cout << "x : " << point.x << std::endl;
      std::cout << "y : " << point.y << std::endl;
    }
    
    point_pub.publish(msg);

}
/*************marker를 markerarray로 push_back하는 함수*************/
void MyPoint::PushBack(visualization_msgs::MarkerArray& s){ //main 문에서 선언한 외부 객체의 레퍼런스를 받아와야 함수가 끝나도 값이 사라지지 않는다.
  
    ReadFile();// 해당 함수를 호출함으로써 선언한 벡터들의 값을 넣어준다.

    for (size_t i = 0; i < xCoordinates.size(); ++i){

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        /**********************마커의 네임스페이스************************/
        marker.ns = "my_namespace";

        /********************* 마커에 할당된 고유 ID**********************/
        marker.id = i;

        /*********The available types are specified in the message definition.********/
        marker.type = visualization_msgs::Marker::CUBE;

        /**********0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall***********/
        marker.action = visualization_msgs::Marker::ADD;

        /****************Pose marker, specified as x/y/z position ***************/
        
        marker.pose.position.x = xCoordinates[i];
        marker.pose.position.y = yCoordinates[i];
        marker.pose.position.z = 0.05;

        /******************* x/y/z/w quaternion orientation.*********************/
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0; //쿼터니언 값(?)
       
        /******************마커의 scale [1,1,1] = 1m x 1m x 1m******************/
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        /***************************객체의 색상*****************************/
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.5;
        marker.color.g = 0.8;
        marker.color.b = 1.0;

        s.markers.push_back(marker);
        //only if using a MESH_RESOURCE marker type:
        // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  }
    
}
void MyPoint::PipeLine(){

  visualization_msgs::MarkerArray marker_array;
  PushBack(marker_array);
	vis_pub.publish( marker_array );
  WayPoint(); 
  Clear();

}; 

int main(int argc, char **argv)
{
  /*****************노드명 초기화*****************/
    ros::init(argc, argv, "waypoint_pub");
    MyPoint point;
    ros::Rate loop_rate(10);
    // bool trigger = true;
  /**********************퍼블리셔로 보낼 msg 선언************************/
    
    // ros::Rate delayed_launch(1);
    // delayed_launch.sleep();s
    
    while(ros::ok()){

     
      point.PipeLine();
      ros::spinOnce();
      loop_rate.sleep();   
    }
    return 0;
}
