#include <vector>
#include <random>
#include <cmath>
#include <iostream>
#include <matplot/matplot.h>
#include <Eigen/Dense>
#include "KalmanFilter.h"

using namespace matplot;

// define  global variables for # of meas/predicted values

const int iX = 0;
const int iV = 1;
const int num_vars = iV + 1;
const int measuredvals_num = num_vars - 1;

// defines the two main updated steps globally, so all functions can use the same one
Eigen::MatrixXd new_x(2,1);
Eigen::MatrixXd new_P;



//    Define Constructor , KalmanFilter
// inside constructor , define the GRV (desired filtered parameters)
    // also same for variances
    // define P matrix
 KalmanFilter::KalmanFilter (float initial_x, float initial_v, float accel_variance){
     this -> x = Eigen::MatrixXd(num_vars,1);
     this -> x(iX) = initial_x;
     this -> x(iV) = initial_v;
     this ->accel_variance = accel_variance;
     this -> P =  Eigen::MatrixXd::Identity(num_vars,num_vars);
     return ;
}



void KalmanFilter::PredictionStep (float dt) { 

    // state extrapolation
    this -> F = Eigen::MatrixXd::Identity(num_vars,num_vars);
    this -> F(iX,iV) = dt;
    new_x = F * this -> x ; // no input control for now, so G * u is zero

    this -> G = Eigen::MatrixXd (num_vars, 1);
    G(iX) = 0.5 * dt* dt;
    G(iV) = dt;

    // define Process Noise Covariance
    Eigen::MatrixXd Q = G * G.transpose() * this -> accel_variance * this-> accel_variance;

    // extrapolate uncertainty

    new_P = F * this -> P * F.transpose() + Q;

    // update for iterations
    this -> P = new_P;
    this -> x = new_x;
    
    return ;
}


void KalmanFilter::UpdateStep(float meas_value, float meas_variance) {
    // obtaining the private variables
    Eigen::MatrixXd ztemp(1,1);
    ztemp << meas_value;
    this -> setmeasvec(ztemp);
    this -> z = this ->getmeasvec();
    Eigen::MatrixXd Htemp(1, 2);
    Htemp << 1 , 0;
    this-> setObsMatrix(Htemp);
    this -> H = this ->getObsMatrix();

    Eigen::MatrixXd R;
     R = Eigen::MatrixXd(measuredvals_num,measuredvals_num);
     R(0) = meas_variance ;
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




