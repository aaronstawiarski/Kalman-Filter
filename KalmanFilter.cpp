#include <vector>
#include <random>
#include <cmath>
#include <iostream>
#include <matplot/matplot.h>
#include <Eigen/Dense>
#include "KalmanFilter.h"

using namespace matplot;

// define  global variables for # of meas/predicted values

// for camera, will start with 2D pos and 2D vel (also estimate and y accel)




// 4 measured vals, but 6 total vals in state vector

const int measuredvals_num = 4;
const int state_num = 6;



// defines the two main updated steps globally, so all functions can use the same one
Eigen::MatrixXd new_x(state_num,1);
Eigen::MatrixXd new_P;



//    Define Constructor , KalmanFilter
// inside constructor , define the GRV (desired filtered parameters)
    // also same for variances
    // define P matrix
 KalmanFilter::KalmanFilter (double pos_x, double pos_y, double vel_x, double vel_y, double accel_x, double accel_y, double jerk_variance){
     this -> x = Eigen::MatrixXd(state_num,1);
             x << pos_x, pos_y, vel_x, vel_y, accel_x, accel_y;
     this -> jerk_variance = jerk_variance;
     this -> P =  Eigen::MatrixXd::Identity(state_num, state_num);
     return ;
}



void KalmanFilter::PredictionStep (float dt, float jerk_variance) { 

    // state extrapolation
    this -> F = Eigen::MatrixXd::Identity(state_num, state_num);
    // work out 6x6 computation by hand
    F(0, 2)= dt;
    F(0, 4) = 0.5 * dt * dt;
    F(1,3) = dt;
    F(1, 5) = 0.5 * dt * dt;
    F(2,4) = dt;
    F(3, 5) = dt;
    new_x = F * this -> x;

    const double step = dt;
    const double dt2 = step * step;
    const double dt3 = dt2 * step;
    G.resize(state_num, 2);
    // The columns map per-step x and y jerk into position, velocity, and acceleration.
    G << dt3 / 6.0, 0.0,
         0.0,       dt3 / 6.0,
         dt2 / 2.0, 0.0,
         0.0,       dt2 / 2.0,
         step,      0.0,
         0.0,       step;

    const Eigen::MatrixXd Q = jerk_variance * (G * G.transpose());

    // extrapolate uncertainty

    new_P = F * this -> P * F.transpose() + Q;

    // update for iterations
    this -> P = new_P;
    this -> x = new_x;
    
    return ;
}


void KalmanFilter::UpdateStep(std::vector<float> meas_value, float meas_variance) {
    // obtaining the private variables
    Eigen::MatrixXd ztemp(4,1); // temp for the give ISS data 
    ztemp << meas_value[0], meas_value[1], meas_value[2], meas_value[3];
    this -> setmeasvec(ztemp);
    this -> z = this ->getmeasvec();
    Eigen::MatrixXd Htemp(1, 6);
    Htemp << 1 , 1, 1, 1, 0, 0;
    this-> setObsMatrix(Htemp);
    this -> H = this ->getObsMatrix();

    Eigen::MatrixXd R;
     R = Eigen::MatrixXd(measuredvals_num,measuredvals_num);
     for(int i = 0; i < 4; i++ ){
     R(i, i) = meas_variance;
     }
    Eigen::MatrixXd y = z - H * (this -> x); 
    Eigen::MatrixXd S = H * (this -> P) * ((this -> H).transpose()) + R ;
    Eigen::MatrixXd K = (this -> P) * ((this -> H).transpose()) * (S.inverse()) ;

    new_x = this -> x + K * y;
    new_P = (Eigen::MatrixXd::Identity(2,2) - K * this -> H) * (this -> P);
    // redefine new = old for iterations
    this -> x = new_x;
    this -> P = new_P;

    return ;

}

// defining the setter and getter funcitons

void KalmanFilter::setmeasvec(Eigen::MatrixXd z_temp){
    this -> z = z_temp;
}


Eigen::MatrixXd KalmanFilter::getmeasvec(){
    return this -> z;
}

        

void KalmanFilter::setObsMatrix(Eigen::MatrixXd H_temp){

    this -> H = H_temp;
}


Eigen::MatrixXd KalmanFilter::getObsMatrix(){

    return this -> H;
}




