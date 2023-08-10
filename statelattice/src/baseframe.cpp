#include "/home/eonsoo/catkin_ws/src/statelattice/include/statelattice/lattice.h"

void BaseFrame::calculateDisance(){
    
    double temp = 0;
    calculated_distance.push_back(0);
    for (int i = 0; i < point_x.size()-1 ; i++) {
        temp = pow((point_x[i+1] - point_x[i]), 2) + pow((point_y[i]+1 - point_y[i]), 2);
        temp = sqrt(temp);
        calculated_distance.push_back(temp);
        std::cout << "calculated distance " << i << " : " << calculated_distance[i] << std::endl;
    }
    
}

void BaseFrame::normalizeDistance() {
    // Assuming the input distance is between 0 and some maximum value
    // You can adjust the maximum value based on your application
    maxDistance = pow((point_x.back() - point_x.front()), 2) + pow((point_y.back() - point_y.front()), 2);  // Adjust this value as needed
    maxDistance = sqrt(maxDistance);
    std::cout << "maxDistance : " << maxDistance << std::endl; 
    // Normalize the distance to the range [0, 1]
    for (int i = 0; i < calculated_distance.size() ; i++) {
        
        normalized_distance.push_back(calculated_distance[i] / maxDistance);
        std::cout << "nomalized distance " << i << " : " << normalized_distance[i] << std::endl;

    }    
}

void BaseFrame::makeMatrix(){

    // for(int i = 0 ; i < calculated_distance.size(); i++){

    //     double s3 = pow(calculated_distance[i], 3);
    //     std::cout << "calcul " << s3 << std::endl;
    //     double s2 = pow(calculated_distance[i], 2);
    //     double s1 = calculated_distance[i];
    //     double s0 = 1.;
    for(int i = 0 ; i < normalized_distance.size(); i++){

        double s3 = pow(normalized_distance[i], 3);
        double s2 = pow(normalized_distance[i], 2);
        double s1 = normalized_distance[i];
        double s0 = 1.;
        
        std::vector<double> s_column;
        
        s_column.push_back(s3);
        s_column.push_back(s2);
        s_column.push_back(s1);
        s_column.push_back(s0);

        s_mat.push_back(s_column);
        x_vec.push_back(point_x[i]);
        y_vec.push_back(point_y[i]);
    }
    
}

void BaseFrame::getMatrixA() {

    makeMatrix();
    

    for (int row = 0; row < MATRIX_ROW ; row++) {
        for (int col = 0; col < MATRIX_COL ; col++) {

            s_matrix(row, col) = s_mat[row][col];
        }
        x_vector(row, 0) = x_vec[row];
        y_vector(row, 0) = y_vec[row];
    }
    arma::mat temp = arma::pinv(s_matrix);
    Ax_matrix = temp * x_vector;
    Ay_matrix = temp * y_vector;

    std::cout << "Ax_Coefficients : " << std::endl <<  Ax_matrix << std::endl;
    std::cout << "Ay_Coefficients : " << std::endl <<  Ay_matrix << std::endl;  
}


void BaseFrame::Do(){

    calculateDisance();
    normalizeDistance();
    getMatrixA();

}

int main() {
    
    BaseFrame base;

    base.Do();

    return 0;
    // // Normalize the distance between 0 and 1
    // double normalizedDistance = normalizeDistance(distance);
    
    // std::cout << "Original Distance: " << distance << std::endl;
    // std::cout << "Normalized Distance: " << normalizedDistance << std::endl;
    
    // return 0;
}