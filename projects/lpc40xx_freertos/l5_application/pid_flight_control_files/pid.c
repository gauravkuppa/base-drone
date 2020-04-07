#include "pid.h"
#include <time.h>
#include <stdbool.h>
#include <stdio.h>
#define MAN 0 // Define if manual or in autonomous mode
#define AUTO 1
bool inAutomatic = false;

void PID_init(PID *pid, float motorRPMHighThreshold,
              float motorRPMLowThreshold) {
  // set these values to a little lower than actual motor RPM values
  pid->motorRPMHighThreshold = motorRPMHighThreshold;
  pid->motorRPMLowThreshold = motorRPMLowThreshold;
  pid->sampleTime = 1000 / 200000; // sampling once per 5 microseconds
  pid->last_filtered_error = 0;
  pid->alpha = 0.4; // the higher the value of alpha, the less depenecy the
                    // filtered_d has on older values
}

void computePID(PID *pid, float expected_state, float measured_state) {
  if (!inAutomatic)
    return; // Manual override
  clock_t now = clock();
  double current_time = ((double) now) / CLOCKS_PER_SEC;
  double time_change = current_time - pid->last_time;

  if (time_change >= pid->sampleTime) {
    double error = expected_state - measured_state; // Setpoint - input/output
    pid->err_sum += error;
    double dErr = pid->input - pid->lastInput; // Leo: Do we have an input
                                               // variable? I can add in the .h

    // Compute the PID Output
    float p, i, d;

    p = pid->k_p * error;
    i = pid->k_i * pid->err_sum;
    d = pid->k_d * dErr;

    pid->measured_state =
        p + i - d; // Derivative kick: dErr/dr is equal to -Dinput/dt
    pid->lastInput = pid->input; // Save last variable's calculation
    pid->last_time = now;        // Save last time

    float filtered_d;

    // derivative filter using exponential moving average -- used to discount
    // noise picked up by derivative path
    if (pid->last_filtered_error == 0) {
      filtered_d = d;
    } else {
      filtered_d = pid->alpha * d + (1 - pid->alpha) * pid->last_filtered_error;
    }

    pid->measured_state = p + i + filtered_d;

    // implement clamping to prevent integral windup
    if (pid->measured_state > pid->motorRPMHighThreshold) {
      pid->measured_state = pid->motorRPMHighThreshold;
    } else if (pid->measured_state < pid->motorRPMLowThreshold) {
      pid->measured_state = pid->motorRPMLowThreshold;
    }

    // keep track of some variables
    pid->last_filtered_error = filtered_d;
    pid->last_error = error;
    pid->last_time = current_time;
  }
}

// Todo:
void setTunings(PID *pid, float input_k_p, float input_k_i, float input_k_d) {
  double sample_time_in_sec =
      ((double)pid->sampleTime) / 200000; // Figure pid conversion out
  pid->k_p = input_k_p;
  pid->k_i = input_k_i * sample_time_in_sec;
  pid->k_d = input_k_d / sample_time_in_sec;
}

// Todo: Figure out sample time
void setSampleTime(PID *pid, float newSampleTime) {
  if (newSampleTime > 0) {
    double ratio = (double)newSampleTime / (double)pid->sampleTime;
    pid->k_i *= ratio;
    pid->k_d /= ratio;
    pid->sampleTime = (unsigned long)newSampleTime;
  }
}
// Basic mode set function
void setMode(int mode) { inAutomatic = (mode == AUTO); }