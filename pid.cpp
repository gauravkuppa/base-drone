#include <stdio.h>
#include <chrono>
#include "pid.h"

using namespace std;

PID::PID(float motorRPMHighThreshold, float motorRPMLowThreshold) {
     // set these values to a little lower than actual motor RPM values
    this->motorRPMHighThreshold = motorRPMHighThreshold;
    this->motorRPMLowThreshold = motorRPMLowThreshold;
}

void PID::tuneParameters(float input_k_p, float input_k_i, float input_k_d) {
    this->k_p = input_k_p;
    this->k_i = input_k_i;
    this->k_d = input_k_d;
}

void PID::computePID(float expected_state, float measured_state) {
    
    auto now = chrono::steady_clock::now();
    double current_time = chrono::duration<double>(now.time_since_epoch()).count();
    double time_change = current_time - last_time;

    float error = expected_state - measured_state;
    this->err_sum += error*time_change;
    float p, i, d;

    p = this->k_p * error;
    i = this->k_d * (err_sum);
    d = this->k_i * (error - last_error)/time_change;
    //bool allowIntegration = true;

    this->measured_state = p + i + d;

    last_error = error;
    last_time = current_time;

    // clamping for anti-windup integration

    /**if (this->current_state > motorRPMHighThreshold) {
        this->current_state = motorRPMHighThreshold;
        if (error > 0) {
            allowIntegration = false;
        }
    } else if (this->current_state < motorRPMLowThreshold) {
        this->current_state = motorRPMLowThreshold;
        if (error < 0) {
            allowIntegration = false;
        } 
    }**/

    // reset current_state
    
    /**if(allowIntegration) {
        this->i = this->k_i * integral(error); // how do i do an integral in C? best way? // also, what is tau value versus time
    } else {
        this->i = 0;
    }**/

    // derivative path enlarges values of noise
    // derivative --> low pass filter
    // use laplace transform function?

    //this->current_state = this->p + this->i + this->d;

}