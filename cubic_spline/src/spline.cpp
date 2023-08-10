#include "/home/eonsoo/catkin_ws/src/cubic_spline/include/spline.h"

Spline::Spline(){}

Spline::~Spline(){}

//g_i(x) 일 때
//g(x_i+1) = ax^3 + bx^2 + cx = eta
//g'(x_i+1) = 3ax^2 + 2bx + c = c
//g''(x_i+1) = 6ax + 2b = 2b        
//not-a-knot
//h[1]*segments[0].b-(h[0]+h[1])*segments[1].b +h[0]*segments[2].b = 0

void Spline::matrixCalculator(){
    temp1.push_back(h[0]);
    temp1.push_back(-(h[0]+h[1]));
    temp1.push_back(h[0]);
    for(int i = 3; i < n+1 ; i++){
        temp1.push_back(0);
    }
    for(int i = 0; i < n+1 ; i++){
        std::cout << temp1[i] << " ";
    }
    A_matrix.push_back(temp1);
    temp1.clear();

    std::cout << "" << std::endl; 
    for(int i = 0; i < n-1 ; i++){
        for(int j = 0; j < n+1 ; j++){
            if(j == i){
                temp1.push_back(h[j]/3);
            }
            else if(j == i+1){
                temp1.push_back((h[j-1]+h[j])*2/3);
            }
            else if(j == i+2){
                temp1.push_back(h[j-1]/3);
            }
            else{       
            temp1.push_back(0); 
            }
            std::cout << temp1[j] << " ";
        }
        A_matrix.push_back(temp1);
        std::cout << "" << std::endl; 
        temp1.clear();
    }
    for(int i = 0; i < n-2 ; i++){
        temp1.push_back(0);
    }
    
    temp1.push_back(h[n-1]);
    temp1.push_back(-(h[n-2]+h[n-1]));
    temp1.push_back(h[n-2]);
    
    for(int i = 0; i < n+1 ; i++){
        std::cout << temp1[i] << " ";
    }
    A_matrix.push_back(temp1);
    temp1.clear();
    
}

void Spline::addDataPoint() {
   
    SplineSegment segment;
    for(int i = 0; i < n+1 ; i++){
        segment.x = x[i];
        segment.d = y[i];
        segments.push_back(segment);
        std::cout << "x : " <<segments[i].x <<"  y : "<< segments[i].d << std::endl;
    }
    std::cout << "----------------------" << std::endl;
    for (int i = 0; i < n; i++) {
        //h_i = x_i+1 - x_i
        //eta_i = y_i+1 - y_i (g(x_i) = y_i = d_i)
        
        h.push_back(segments[i + 1].x - segments[i].x);
        std::cout << "h : " << h[i] ;
        eta.push_back(segments[i + 1].d - segments[i].d);
        temp2.push_back(eta[i]);
        std::cout << "  eta : " << eta[i] << std::endl;
    }
    b_matrix.push_back(temp2);
    std::cout << "----------------------" << std::endl;
}

void Spline::computeSplineCoeffficients(){


}

void Spline::Do(){
    addDataPoint();
    matrixCalculator();
}

int main(){

    Spline cubic;
    cubic.Do();

}
