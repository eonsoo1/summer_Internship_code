#include <iostream>
#include <vector>
#include <cmath>
#include <armadillo>


#define MATRIX_ROW 5
#define MATRIX_COL 4

class BaseFrame{
    
    private :
        struct SplineSegment {
            double a, b, c, d, x;
        };  

        //parameter to nomalize the distance
        std::vector<double> point_x;
        std::vector<double> point_y;
        std::vector<double> calculated_distance;
        std::vector<double> normalized_distance;

        //parameter to use psedo inverse
        // x = s_matrix * Ax_matrix 
        // y = s_matrix * Ay_matrix        
        std::vector<std::vector<double>> s_mat;
        std::vector<double> x_vec;
        std::vector<double> y_vec;
       

        arma::mat s_matrix;
        arma::mat x_vector;
        arma::mat y_vector;
        arma::mat Ax_matrix;
        arma::mat Ay_matrix;     // what I want to know for curve fitting

        double maxDistance;

    public :
        BaseFrame() : s_matrix(MATRIX_ROW, MATRIX_COL),
            x_vector(MATRIX_ROW, 1),
            y_vector(MATRIX_ROW, 1),
            Ax_matrix(4, 1),
            Ay_matrix(4, 1) {

            for(int i = 0; i < 100; i++ ){
                point_x.push_back(i+5);
                point_y.push_back(i-4);
            }

            };
        ~BaseFrame(){};
        void calculateDisance();
        void normalizeDistance();
        void makeMatrix();
        void getMatrixA();
        void Do();
};