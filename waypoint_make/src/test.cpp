#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

std::vector<std::string> Split(std::string input, char delimeter){
    
    std::vector<std::string> oneline;
    std::istringstream ss(input);
    std::string output;
    
    double x;

    while(getline(ss, output, delimeter)){
        
        oneline.push_back(output);

    }
    return oneline;
}

void MakeArray(std::vector<double> waypoint_double){ 
    std::vector<double> xCoordinates;
    std::vector<double> yCoordinates;
    int count = 0;
    std::vector<double>::iterator iter;
    for(iter = waypoint_double.begin(); iter!= waypoint_double.end(); iter++){
       if(count%2 == 0){

        xCoordinates.push_back(*iter);

       }
       else{
       
       yCoordinates.push_back(*iter);
       
       }      
       count++;
    }
    std::vector<double>::iterator iter_x;
    for(iter_x = xCoordinates.begin(); iter_x!= xCoordinates.end(); iter_x++){
       std::cout << "Iterator x " << *iter_x << std::endl; 
    }
    std::vector<double>::iterator iter_y;
    for(iter_y = xCoordinates.begin(); iter_y!= xCoordinates.end(); iter_y++){
       std::cout << "Iterator y " << *iter_y << std::endl; 
    }
   
   
 

}
int main() {

  std::vector<std::string> waypoint;
  std::vector<std::string> waypoint_temp;
  std::vector<double> waypoint_double;
  // 파일 읽기 준비
  std::ifstream in("waypoint_2nd.txt");
  
  

  if (!in.is_open()) {
    std::cout << "파일을 찾을 수 없습니다!" << std::endl;
    return 0;
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
      std::cout << "Iterator" << *iter_once << std::endl; 
    }

  
  
  std::vector<double>::iterator iter_other;
  for(iter_other = waypoint_double.begin(); iter_other!= waypoint_double.end(); iter_other++){
       std::cout << "Iterator double " << *iter_other << std::endl; 
    }
  std::vector<double> xCoordinates;
  std::vector<double> yCoordinates;
    int count = 0;
    std::vector<double>::iterator iter;
    for(iter = waypoint_double.begin(); iter!= waypoint_double.end(); iter++){
       if(count%2 == 0){

        xCoordinates.push_back(*iter);

       }
       else{
       
       yCoordinates.push_back(*iter);
       
       }      
       count++;
    }
    std::vector<double>::iterator iter_x;
    for(iter_x = xCoordinates.begin(); iter_x!= xCoordinates.end(); iter_x++){
       std::cout << "Iterator x " << *iter_x << std::endl; 
    }
    std::vector<double>::iterator iter_y;
    for(iter_y = yCoordinates.begin(); iter_y!= yCoordinates.end(); iter_y++){
       std::cout << "Iterator y " << *iter_y << std::endl; 
    }
  
}