// TODO: Motor class
float motors[4];

void motor_mixing_algo(float thrust, float roll, float pitch, float yaw) {
    motors[0] = thrust + roll + pitch + yaw;
    motors[1] = thrust - roll + pitch - yaw;
    motors[2] = thrust + roll - pitch - yaw;
    motors[3] = thrust - roll - pitch + yaw;
}

void thrust_pid(float altitude) {
    
}

void roll_pid(float roll) {

}

void pitch_pid(float pitch) {
    
}

void yaw_pid(float yaw) {
    
}

void main () {
    return;
}