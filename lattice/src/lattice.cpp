#include "/home/eonsoo/catkin_ws/src/lattice/include/lattice.h"

void lattice::makePoint(){

    for(int i = 0; i < 100 ; i++){
        point_x.push_back(i);
        point_y.push_back(i);
        //std::cout << point_x[i] << "   " << point_y[i] << std::endl;
    }

}

void lattice::findCenterPoint(){

    for(int i = 0; i < point_x.size() ; i++){
        if( i == point_number){

            center_point_x = point_x[i];
            center_point_y = point_y[i];

            
            std::cout<< "center point = (" << center_point_x << ", " << center_point_y <<")" << std::endl; 
            center_point_tangent = (point_y[i+1] - point_y[i])/(point_x[i+1]-point_x[i]);
            std::cout << "center point tangent = " << center_point_tangent << std::endl;
        }
    }

}

void lattice::findSidePoint(){

    vertical_tan = -(1/center_point_tangent);
    std::cout << "vertical tangent = " << vertical_tan << std::endl;
    
    side_x = sqrt(pow(offset, 2)/ (pow(vertical_tan, 2) + 1)) + center_point_x;
    side_y = vertical_tan * (side_x - center_point_x) + center_point_y ;
    std::cout<< "center point = (" << side_x << ", " << side_y <<")" << std::endl; 

    side_x = -(sqrt(pow(offset, 2)/ (pow(vertical_tan, 2) + 1))) + center_point_x;
    side_y = vertical_tan * (side_x - center_point_x) + center_point_y ;
    std::cout<< "center point = (" << side_x << ", " << side_y <<")" << std::endl; 
}

void lattice::Do(){

    makePoint();
    findCenterPoint();
    findSidePoint();
}

int main(int argc, char**argv){

    //ros::init(argc, argv, "local_planng");
    lattice l;

    l.Do();
    
    return 0;
}