#include <vector>
#include <random>
#include <cmath>
#include <iostream>
#include <matplot/matplot.h>
#include <Eigen/Dense>

    // ideally, would have measurements not be seen along with observation matrix H
    class KalmanFilter {
        
        public:
         KalmanFilter(float initial_x, float initial_y, float accel_variance);
        //defining all needed vectors
        Eigen::MatrixXd  x ;
        Eigen::MatrixXd  P ;
        Eigen::MatrixXd F ;
        Eigen::MatrixXd G ;
        
        
        // setters and getter functions for private vairables
        void setmeasvec(Eigen::MatrixXd z_temp);
        Eigen::MatrixXd getmeasvec();

        void setObsMatrix(Eigen::MatrixXd H_temp);
        Eigen::MatrixXd getObsMatrix();

        // define the prediction and update functions

        void PredictionStep(float dt); // desired time step

        void UpdateStep(float meas_value, float meas_variance); //  inputs are z and R, respectively

        private:

        Eigen::MatrixXd z ;
        Eigen::MatrixXd H;
        float accel_variance;
    };