#include <vector>
#include <random>
#include <cmath>
#include <iostream>
#include <matplot/matplot.h>
#include <Eigen/Dense>

    // ideally, would have measurements not be seen along with observation matrix H
    class KalmanFilter {
        
        public:
         KalmanFilter(double pos_x, double pos_y, double vel_x, double vel_y, double accel_x, double accel_y, double jerk_variance);
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

        void PredictionStep(float dt, float jerk_variance); // desired time step

        void UpdateStep(std::vector<float> meas_value, float meas_variance); //  inputs are z and R, respectively

        private:

        Eigen::MatrixXd z ;
        Eigen::MatrixXd H;
        double jerk_variance;
    };
