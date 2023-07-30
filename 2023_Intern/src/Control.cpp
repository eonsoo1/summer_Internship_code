#include "/home/eonsoo/catkin_ws/src/2023_Intern/include/Control.h"

/******************생성자*******************/

Pure::Pure(){
  speed_sub = n.subscribe("/carla/ego_vehicle/speedometer", 100, &Pure::SpeedCallback, this);
  MyInfo_sub = n.subscribe("/carla/ego_vehicle/odometry", 100, &Pure::EgoVehicleCallback, this);
  waypoint_sub = n.subscribe("point_msgs", 100, &Pure::PointCallback, this);
  controller_pub = n.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 100);
  vis_pub = n.advertise<visualization_msgs::Marker>( "waypoint", 1000 );
  vehicle_vis_pub = n.advertise<visualization_msgs::Marker>( "vehicle_waypoint", 1);
    
}

/**********Speed 값 콜백 하는 함수***********/

void Pure::SpeedCallback(const std_msgs::Float32::ConstPtr& msg){

  vehicle_speed = msg -> data * 3.6;
  
}
 
/**********waypoint 좌표값 받는 함수***********/
void Pure::PointCallback(const waypoint_make::waypoint_msg::ConstPtr& msg){
   
   if(trigger1 == true){
        
    for(int i = 0; i < msg->points.size() ; i++ ){
      XCoordinates.push_back(msg -> points[i].x);//[points vector size 로 for 문 돌리기]
      YCoordinates.push_back(msg -> points[i].y);  
      std::cout<< "x = " << XCoordinates[i] << "    ";
      std::cout<< "y = " << YCoordinates[i] << std::endl;
    } 
    trigger1 = false;
    current_target_x = XCoordinates[0];
    current_target_y = YCoordinates[0];
  } 
}

/*************차량의 현재 값 받는 함수**************/
void Pure::EgoVehicleCallback(const nav_msgs::Odometry::ConstPtr& msg){
    
    vehicle_x = msg-> pose.pose.position.x;
    vehicle_y = msg-> pose.pose.position.y;
    vehicle_z = msg-> pose.pose.position.z;
    orientation_x = msg-> pose.pose.orientation.x;
    orientation_y = msg-> pose.pose.orientation.y;
    orientation_z = msg-> pose.pose.orientation.z;
    orientation_w = msg-> pose.pose.orientation.w;

}
/*************현재 바라보고 있는 target point visualiazation**************/
void Pure::WayPoint(){

  visualization_msgs::Marker marker;

	marker.header.frame_id = "map";

	marker.header.stamp = ros::Time();

  /**********************마커의 네임스페이스************************/
	marker.ns = "my_namespace";

  /********************* 마커에 할당된 고유 ID**********************/
	marker.id = 0;

  /*********The available types are specified in the message definition.********/
	marker.type = visualization_msgs::Marker::CUBE;

  /**********0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall***********/
	marker.action = visualization_msgs::Marker::ADD;

  /****************Pose marker, specified as x/y/z position ***************/
	marker.pose.position.x = current_target_x;
	marker.pose.position.y = current_target_y;
	marker.pose.position.z = current_target_z;

  /******************* x/y/z/w quaternion orientation.*********************/
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0; //쿼터니언 값(?)
	
  /******************마커의 scale [1,1,1] = 1m x 1m x 1m******************/
  marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;

  /***************************객체의 색상*****************************/
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	//only if using a MESH_RESOURCE marker type:
	// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  
	vis_pub.publish( marker );

}

/*************현재 내 차량의 위치 visualiazation**************/

void Pure::VehicleCurrentPoint(){


	line_strip.header.frame_id = "map";

	line_strip.header.stamp = ros::Time::now();

  /**********************마커의 네임스페이스************************/
	line_strip.ns = "line_strip";

  /********************* 마커에 할당된 고유 ID**********************/
	line_strip.id = 0;

  /*********The available types are specified in the message definition.********/
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  /**********0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall***********/
	line_strip.action = visualization_msgs::Marker::ADD;

  /******************* x/y/z/w quaternion orientation.*********************/
	line_strip.pose.orientation.x = 0.0;
	line_strip.pose.orientation.y = 0.0;
	line_strip.pose.orientation.z = 0.0;
	line_strip.pose.orientation.w = 1.0; //쿼터니언 값(?)
	
  /******************마커의 scale [1,1,1] = 1m x 1m x 1m******************/
  line_strip.scale.x = 0.3;
	

  /***************************객체의 색상*****************************/
	line_strip.color.a = 1.0; // Don't forget to set the alpha!
	line_strip.color.r = 1.0;
	line_strip.color.g = 1.0;
	line_strip.color.b = 0.0;

	//only if using a MESH_RESOURCE marker type:
	// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";


 /****************Pose marker, specified as x/y/z position ***************/

  geometry_msgs::Point p1;
  
  p1.x = vehicle_x;
  p1.y = vehicle_y;
  p1.z = vehicle_z;
  
  line_strip.points.push_back(p1);

	vehicle_vis_pub.publish(line_strip);

}

/***********다음 target point 찾는 함수************/

void Pure::NextWaypoint(){

  double ld_next = sqrt(pow((XCoordinates[index]-vehicle_x), 2) + pow((YCoordinates[index]-vehicle_y), 2));

  if(ld_next >= ld){
    current_target_x = XCoordinates[index];
    current_target_y = YCoordinates[index];
    std::cout << "-----next_ld = "<< ld_next <<" 다음 point를 찾았습니다.-----" << std::endl;
    std::cout << " Next_Target_sx = " << current_target_x << std::endl;
    std::cout << " Next_Target_y = " << current_target_y << std::endl; 
  }
  while(ld_next < ld){

    index++;

    current_target_x = XCoordinates[index];
    current_target_y = YCoordinates[index];
    ld_next = sqrt(pow((current_target_x-vehicle_x), 2) + pow((current_target_y-vehicle_y), 2)); 
    std::cout << "-----next_ld = "<< ld_next <<" 다음 point를 찾는 중입니다.-----" << std::endl;
    std::cout << " Next_Target_x = " << current_target_x << std::endl;
    std::cout << " Next_Target_y = " << current_target_y << std::endl;

  }
    
  }

/**************쿼터니언 값을 오일러 값으로 변환하는 함수*****************/

void Pure::GetRPY(){

  tf::Quaternion q(orientation_x, orientation_y, orientation_z, orientation_w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

}

/****************차와 target point 사이의 각도 구하는 함수*****************/

void Pure::AlphaCalculator(){

  GetRPY();

  std::cout << " Vehicle_yaw = " << yaw << std::endl;
  std::cout << " Current_Target_x = " << current_target_x << std::endl;
  std::cout << " Current_Target_y = " << current_target_y << std::endl;
  std::cout << " Current_Vehicle_x = " << vehicle_x << std::endl;
  std::cout << " Current_Vehicle_y = " << vehicle_y << std::endl; 
  alpha = atan2((current_target_y - vehicle_y), (current_target_x - vehicle_x)) - yaw;
  look_ahead_distance = sqrt(pow((current_target_x-vehicle_x), 2) + pow((current_target_y-vehicle_y), 2));

  std::cout<< " Look_Ahead_Distance = " << look_ahead_distance << std::endl;
  std::cout << "" << std::endl;  
}

/**************Pure Pursuit 알고리즘으로 steer 구하는 함수*****************/

double Pure::PureCalculator(){
  
  AlphaCalculator();
  
  double steering_angle = -atan2(2.0 * wheel_base * sin(alpha), look_ahead_distance)*2/PI;
  
    if(look_ahead_distance <= ld){//가까이 오면 다시 다음 포인트 찾기

      std::cout << "-------------다음 waypoint를 찾습니다.--------------" << std :: endl;
      index ++;
      std::cout << " Index : " << index << std :: endl;
      NextWaypoint();
    }
    
    else if(look_ahead_distance > ld){// 멀리 있으면 가까이 올 때까지 대기
      std::cout << "-------------해당 waypoint를 따라갑니다.-------------" << std :: endl;
      std::cout << " Index : " << index << std :: endl;
      std::cout << "" << std::endl;  
      std::cout << "" << std::endl;  
      std::cout << "" << std::endl;  
    }
    
 
  return steering_angle;

}

  /*****************publish 해주는 함수들을 모아둔 함수***************/
void Pure::PublishCommend(carla_msgs::CarlaEgoVehicleControl& msg){

  WayPoint();
  VehicleCurrentPoint();
  controller_pub.publish(msg);

}

/*************PID 제어**************/

double Pure::ControlPid(){
 
  double setspeed = 35;
  double error = setspeed - vehicle_speed;
  double previous_error = 0;
  std::cout<< " Vehicle_speed(km/h) = " << vehicle_speed << std::endl;
  double kp = 0.1;
  double ki = 0.2;
  double kd = 0.05;

  double dt = 0.1;
  double intergral; 
  double differential;

  intergral = intergral + error * dt;
  differential = error - previous_error / dt;
  
  vehicle_speed = kp * error + ki * intergral - kd * differential;
  if (vehicle_speed > 1){
      vehicle_speed = 1 ;
  } 
  previous_error = error;
  

  return vehicle_speed;

}
double Pure::CalculateCTE(){

    // Calculate the distance between the car and the waypoint
    double dx = vehicle_x - current_target_x;
    double dy = vehicle_y - current_target_y;

    // Calculate the cross track error (signed distance)
    double cte = (dx * sin(yaw)) - (dy * cos(yaw));
    
    return cte;
}
/************main문에서 실행되어야할 함수들을 모아둔 함수**************/

void Pure::Do(){
  
  carla_msgs::CarlaEgoVehicleControl msg;
  if(index < XCoordinates.size()-5){
    msg.throttle = ControlPid();
    msg.steer = PureCalculator();    
  } 
  else if(index >= XCoordinates.size()-5){
    msg.throttle = 0;
    msg.brake = 1;
  }
  std::cout<< " Throttle(PID) = " << msg.throttle << "   ";
  std::cout<< " Steering = " << msg.steer << std::endl;
  PublishCommend(msg); // publish 함수 호출
  std::cout << "" << std::endl; 
  std::cout<< " ERROR = " << CalculateCTE() << std::endl;
}

int main(int argc, char**argv){

    ros::init(argc, argv, "topic_control");
    Pure p;
    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()){
      std::cout << "" << std::endl;  
      std::cout<< "====================================================" << std::endl;  
      std::cout << "" << std::endl;  
      if(count > 5){
        p.Do();
      }
      ros::spinOnce();
      loop_rate.sleep();
      if(count < 10){
         count++;
      }
    }
    return 0;

}
