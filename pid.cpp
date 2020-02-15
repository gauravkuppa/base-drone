#include <stdio.h>
#include "pid.h"

PID::PID(float motorRPMHighThreshold, float motorRPMLowThreshold) {
    this->motorRPMHighThreshold = motorRPMHighThreshold;
    this->motorRPMLowThreshold = motorRPMLowThreshold;
}

void PID::tuneParameters(float input_k_p, float input_k_i, float input_k_d) {
    this->k_p = input_k_p;
    this->k_i = input_k_i;
    this->k_d = input_k_d;
}

void PID::updatePID(float expected_state, float measured_state) {
    float motorRPMHighThreshold, motorRPMLowThreshold; // set these values to a little lower than actual motor RPM values
    bool allowIntegration = true;

    float error = expected_state - measured_state;

    // clamping for anti-windup integration

    if (this->current_state > motorRPMHighThreshold) {
        this->current_state = motorRPMHighThreshold;
        if (error > 0) {
            allowIntegration = false;
        }
    } else if (this->current_state < motorRPMLowThreshold) {
        this->current_state = motorRPMLowThreshold;
        if (error < 0) {
            allowIntegration = false;
        } 
    }

    // reset current_state
    this->current_state = 0;
    
    this->p = this->k_p * error;
    if(allowIntegration) {
        this->i = this->k_i * integral(error); // how do i do an integral in C? best way?
    } else {
        this->i = 0;
    }

    // derivative path enlarges values of noise
    // derivative --> low pass filter
    // use laplace transform function?
    this->d = this->k_d * derivative(error); // how do i do a derivative in C, best way? efficiency?

    this->current_state = this->p + this->i + this->d;

}