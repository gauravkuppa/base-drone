#include <stdio.h>
#include <chrono>
#include "pid.h"

using namespace std;

PID::PID(float motorRPMHighThreshold, float motorRPMLowThreshold) {
     // set these values to a little lower than actual motor RPM values
    this->motorRPMHighThreshold = motorRPMHighThreshold;
    this->motorRPMLowThreshold = motorRPMLowThreshold;
}
<<<<<<< HEAD

void PID::tuneParameters(float input_k_p, float input_k_i, float input_k_d) {
    this->k_p = input_k_p;
    this->k_i = input_k_i;
    this->k_d = input_k_d;
}

void PID::computePID(float expected_state, float measured_state) {
    unsigned long lastTime;
    auto now = chrono::steady_clock::now();
    double current_time = chrono::duration<double>(now.time_since_epoch()).count();
    double time_change = current_time - last_time;
    int sampleTime = 200000;    //Leo: Example sample time in code, pi sample rate is 200KHz 5 µS
                                //Figure out the sample rate we need

    if(time_change >= sampleTime)
    {
        double error = expected_state - measured_state; //Setpoint - input?
        this->err_sum += error*time_change; //Why multiplied by time change?
        double dErr = error - last_error;

        //Compute the PID Output
        float p, i, d;

        p = this->k_p * error;
        i = this->k_i * (err_sum);
        d = this->k_d * (error - last_error)/time_change;
        //bool allowIntegration = true;

        this->measured_state = p + i + d;        

    }
   
    


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
=======

void PID::tuneParameters(float input_k_p, float input_k_i, float input_k_d) {
    this->k_p = input_k_p;
    this->k_i = input_k_i;
    this->k_d = input_k_d;
}

void PID::computePID(float expected_state, float measured_state) {
    unsigned long lastTime; // what is this
    auto now = chrono::steady_clock::now();
    double current_time = chrono::duration<double>(now.time_since_epoch()).count();
    double time_change = current_time - last_time;
    int sampleTime = 200000;    //Leo: Example sample time in code, pi sample rate is 200KHz 5 µS
                                //Figure out the sample rate we need

    if(time_change >= sampleTime)
    {
        double error = expected_state - measured_state; //Setpoint - input?
        this->err_sum += error*time_change; //Why multiplied by time change?
        double dErr = error - last_error;

        //Compute the PID Output
        float p, i, d;

        p = this->k_p * error;
        i = this->k_i * (err_sum);
        d = this->k_d * (error - last_error)/time_change;
        //bool allowIntegration = true;

        this->measured_state = p + i + d;        

        float filtered_d;

        // derivative filter using exponential moving average -- used to discount noise picked up by derivative path
        if (this->last_filtered_error == NULL) {
            filtered_d = d;
        } else {
            filtered_d = this->alpha * d + (1 - this->alpha) * this->last_filtered_error;
        }
        

        this->measured_state = p + i + filtered_d;
        
        // implement clamping to prevent integral windup
        if(this->measured_state > this->motorRPMHighThreshold) {
            this->measured_state = this->motorRPMHighThreshold;
        } else if(this->measured_state < this->motorRPMLowThreshold) {
            this->measured_state = this->motorRPMLowThreshold;
        }
>>>>>>> 07d1c0fe20b337239da00f206956e6abc78f6ce7

    // reset current_state
    
    /**if(allowIntegration) {
        this->i = this->k_i * integral(error); // how do i do an integral in C? best way? // also, what is tau value versus time
    } else {
        this->i = 0;
    }**/

<<<<<<< HEAD
    // derivative path enlarges values of noise
    // derivative --> low pass filter
    // use laplace transform function?

    //this->current_state = this->p + this->i + this->d;

=======
        // keep track of some variables
        this->last_filtered_error = filtered_d;
        this->last_error = error;
        this->last_time = current_time;

    }

    
    
    

>>>>>>> 07d1c0fe20b337239da00f206956e6abc78f6ce7
}