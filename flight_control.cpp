#include "pid.cpp"


// TODO: Motor 

float motors[4];

void motor_mixing_algo(float thrust, float roll, float pitch, float yaw) {
    motors[0] = thrust + roll + pitch + yaw;
    motors[1] = thrust - roll + pitch - yaw;
    motors[2] = thrust + roll - pitch - yaw;
    motors[3] = thrust - roll - pitch + yaw;
}

float thrust_pid(float altitude, float input_k_p, float input_k_i, float input_k_d) {
    PID *pid = new PID(0, 1000);
    pid->setTunings(input_k_p, input_k_i, input_k_d);
    pid->computePID(100, altitude);
    return pid->measured_state;
}

float roll_pid(float roll, float input_k_p, float input_k_i, float input_k_d) {
    PID *pid = new PID(0, 1000);
    pid->setTunings(input_k_p, input_k_i, input_k_d);
    pid->computePID(100, roll);
    return pid->measured_state;

}

float pitch_pid(float pitch, float input_k_p, float input_k_i, float input_k_d) {
    PID *pid = new PID(0, 1000);
    pid->setTunings(input_k_p, input_k_i, input_k_d);
    pid->computePID(100, pitch);
    return pid->measured_state;
    
}

float yaw_pid(float yaw, float input_k_p, float input_k_i, float input_k_d) {
    PID *pid = new PID(0, 1000);
    pid->setTunings(input_k_p, input_k_i, input_k_d);
    pid->computePID(100, yaw);
    return pid->measured_state;
    
}

void main () {
    return;
}