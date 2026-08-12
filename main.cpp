#include <vector>
#include <random>
#include <cmath>
#include <iostream>
#include <matplot/matplot.h>
#include <Eigen/Dense>
#include "KalmanFilter.h"
#include <list>
#include <algorithm>

using namespace matplot;


//define the Kalman Filter class
//define timestep, # of steps, and what step interval
//define initial pos, vel, & measured variance
//define state mus, covariacne matrices, true pos & vel
// iterate through # of desired time steps
    //update x = x + dt*v
    // if step isnt zero and steps divisible by desired time step, update w/ rand noise
    // variable.push_back(quantity you want added) the values on initialized lists

// plot w/ uncertainty bounds, could be 2 standard deviations each way

// defining global variables, so scope is throughout program in main


const float dt = 0.1;
const int num_steps = 20;
const int total_steps = 1000;

float real_x = 0.0;
float real_v = 0.5;
float meas_variance= 0.1 * 0.1;
std::vector <Eigen::MatrixXd> mus = {};
std::vector <Eigen::MatrixXd> covs = {};
std::list <float> real_xs ={};
std::list <float> real_vs = {};

// bound creation for plotting
std::list <float> lower_bound_pos ={};
std::list <float> upper_bound_pos={};

std::list <float> lower_bound_vel ={};
std::list <float> upper_bound_vel={};

// define arrays in order to store the state estimate
 std::vector <float> mus_pos;
 std::vector <float> mus_vel;


int main () {
    //random seed declaration
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> normal(0.0f, 1.0f); //random gaussian distribution
    // declaration of kalman filter class
    KalmanFilter kf = KalmanFilter(0.0f, 1.0f, 0.1f);



    // starting update loop
    for(int i = 0; i< total_steps; i++){
        covs.push_back(kf.P);
        mus.push_back(kf.x);
        kf.PredictionStep(dt);
        real_x = real_x + dt * real_v;
        if(i!=0 && i % num_steps == 0){ // preps update functioncall
            kf.UpdateStep(real_x + normal(gen) * std::sqrt(meas_variance), meas_variance) ;
        }
        // append the measued variables to a list to store for analysis
        real_xs.push_back(real_x);
        real_vs.push_back(real_v);
        lower_bound_pos.push_back(kf.x(0) - 2*sqrt(kf.P(0,0)));
        lower_bound_vel.push_back(kf.x(1) - 2*sqrt(kf.P(1,1)));
        upper_bound_pos.push_back(kf.x(0) + 2*sqrt(kf.P(0,0)));
        upper_bound_vel.push_back(kf.x(1) + 2*sqrt(kf.P(1,1)));
        mus_pos.push_back(kf.x(0));
        mus_vel.push_back(kf.x(1));
    }

// create the bounds by iter


// now, plot all of the data
  


// POSITION

figure();
title("Position versus time");
xlabel("Time (s)");
ylabel("Position (m)");
plot( real_xs, "b-");
hold( on);
plot( mus_pos, "r-");
plot( lower_bound_pos, "r--");
plot( upper_bound_pos, "r--");
legend({"True Position", "Estimated Position", "-2σ", "+2σ"});

// VELOCITY
figure();
title( "Velocity versus time");
xlabel( "Time (s)");
ylabel( "Velocity (m/s)");
plot( real_vs, "b-");
hold( on);
plot( mus_vel, "r-");
plot( lower_bound_vel, "r--");
plot( upper_bound_vel, "r--");
legend({"True Velocity", "Estimated Velocity", "-2σ", "+2σ"});


show();

return 0;
}




