#include <stdio.h>
#include <chrono>
#include "pid.h"

using namespace std;

PID::PID(float motorRPMHighThreshold, float motorRPMLowThreshold) {
     // set these values to a little lower than actual motor RPM values
    this->motorRPMHighThreshold = motorRPMHighThreshold;
    this->motorRPMLowThreshold = motorRPMLowThreshold;
}

void PID::computePID(float expected_state, float measured_state) {
    auto now = chrono::steady_clock::now();
    double current_time = chrono::duration<double>(now.time_since_epoch()).count();
    double time_change = current_time - this->last_time;

    if(time_change >= sampleTime)
    {
        double error = expected_state - measured_state; //Setpoint - input/output
        this->err_sum += error;
        double dErr = input - this->lastInput; //Leo: Do we have an input variable? I can add in the .h

        //Compute the PID Output
        float p, i, d;

        p = this->k_p * error;
        i = this->k_i * err_sum;
        d = this->k_d * dErr;

        this->measured_state = p + i - d; //Derivative kick: dErr/dr is equal to -Dinput/dt        
        this->lastInput = this->input; //Save last variable's calculation
        this->last_time = now; //Save last time 

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

        // keep track of some variables
        this->last_filtered_error = filtered_d;
        this->last_error = error;
        this->last_time = current_time;
    }
}

//Todo:
void PID::setTunings(float input_k_p, float input_k_i, float input_k_d){
    double sample_time_in_sec = ((double)sampleTime)/200000; //Figure this conversion out
    this->k_p = input_k_p;
    this->k_i = input_k_i*sample_time_in_sec;
    this->k_d = input_k_d/sample_time_in_sec;

}

//Todo: Figure out sample time
void PID::setSampleTime(float NewSampleTime){
    if(NewSampleTime > 0){
        double ratio = (double) NewSampleTime/(double)sampleTime;
        this->k_i *= ratio;
        this->k_d /= ratio;
        sampleTime = (unsigned long) NewSampleTime;
    }
}