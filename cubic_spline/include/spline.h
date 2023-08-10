#include <iostream>
#include <vector>
#include <cmath>

class Spline{
    
    private :
        struct SplineSegment {
            double a, b, c, d, x;
        };  
        std::vector<double> x = {0.0, 1.0, 2.0, 3.0};
        std::vector<double> y = {0.0, 1.0, 1.0, 2.0};
        std::vector<SplineSegment> segments;
        std::vector<double> h;
        std::vector<double> eta;
        std::vector<double> temp1;
        std::vector<double> temp2;        
        std::vector<std::vector<double>> A_matrix;
        std::vector<std::vector<double>> b_matrix;
        int n = x.size()-1;


    public :
        Spline();
        ~Spline();
        void matrixCalculator();
        void addDataPoint();
        void computeSplineCoeffficients();
        void Do();

};