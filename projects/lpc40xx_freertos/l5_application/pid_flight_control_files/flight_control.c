#include "flight_control.h"
#include "pid.h"

// TODO: Motor
// TODO: create makefile
float motors[4];

void motor_mixing_algo(float thrust, float roll, float pitch, float yaw) {
  motors[0] = thrust + roll + pitch + yaw;
  motors[1] = thrust - roll + pitch - yaw;
  motors[2] = thrust + roll - pitch - yaw;
  motors[3] = thrust - roll - pitch + yaw;
}

float thrust_pid(float altitude, float input_k_p, float input_k_i,
                 float input_k_d) {
  PID *pid;
  PID_init(pid, 0, 1000 * 14.7); // high RPM = kV * battery Voltage [a little
                                 // lower than actual value]
  setTunings(pid, input_k_p, input_k_i, input_k_d);
  computePID(pid, 100, altitude);
  return pid->measured_state;
}
float roll_pid(float roll, float input_k_p, float input_k_i, float input_k_d) {
  PID *pid;
  PID_init(pid, 0, 1000 * 14.7); // high RPM = kV * battery Voltage [a little
                                 // lower than actual value]
  setTunings(pid, input_k_p, input_k_i, input_k_d);
  computePID(pid, 100, roll);
  return pid->measured_state;
}

float pitch_pid(float pitch, float input_k_p, float input_k_i,
                float input_k_d) {
  PID *pid;
  PID_init(pid, 0, 1000 * 14.7); // high RPM = kV * battery Voltage [a little
                                 // lower than actual value]
  setTunings(pid, input_k_p, input_k_i, input_k_d);
  computePID(pid, 100, pitch);
  return pid->measured_state;
}

float yaw_pid(float yaw, float input_k_p, float input_k_i, float input_k_d) {
  PID *pid;
  PID_init(pid, 0, 1000 * 14.7); // high RPM = kV * battery Voltage [a little
                                 // lower than actual value]
  setTunings(pid, input_k_p, input_k_i, input_k_d);
  computePID(pid, 100, yaw);
  return pid->measured_state;
}

/**RotationMatrix * euler_to_rot_mat(float roll, float pitch, float yaw) {

}

Quaternion * rot_mat_to_quat(RotationMatrix * rot_mat) {

}

RotationMatrix * _world_to_body_translation(RotationMatrix* rot_mat) {

}**/

/**Quaternion * _world_to_body_rotation(float * rotational_axis, float theta,
float * current_vector) {



    Quaternion *q;
    Quaternion *q_star;
    Quaternion *vector;
    Quaternion *q_vector;
    Quaternion *q_vector_q_star;

    // TODO: implement lookup table for cos, sin

    q->a = cos(theta/2);
    q->b = sin(theta/2)*rotational_axis[0];
    q->c = sin(theta/2)*rotational_axis[1];
    q->d = sin(theta/2)*rotational_axis[2];

    q_star->a = cos(theta/2);
    q_star->b = -1 * sin(theta/2)*rotational_axis[0];
    q_star->c = -1 * sin(theta/2)*rotational_axis[1];
    q_star->d = -1 * sin(theta/2)*rotational_axis[2];

    vector->a = 0;
    vector->b = current_vector[0];
    vector->c = current_vector[1];
    vector->d = current_vector[2];

    // calculate q * vector * q_star ( quarternion multiplication )

    **
     *
    (Q1 * Q2).a = (a1a2 - b1b2 - c1c2 - d1d2)
    (Q1 * Q2).b = (a1b2 + b1a2 + c1d2 - d1c2)
    (Q1 * Q2).c = (a1c2 - b1d2 + c1a2 + d1b2)
    (Q1 * Q2).d = (a1d2 + b1c2 - c1b2 + d1a2)
     *
    **

    q_vector->a = q->a*vector->a - q->b*vector->b - q->c*vector->c -
q->d*vector->d; q_vector->b = q->a*vector->b + q->b*vector->a + q->c*vector->d -
q->d*vector->c; q_vector->c = q->a*vector->c - q->b*vector->d + q->c*vector->a +
q->d*vector->b; q_vector->d = q->a*vector->d + q->b*vector->c - q->c*vector->b +
q->d*vector->a;


    q_vector_q_star->a = q_vector->a*vector->a - q_vector->b*vector->b -
q_vector->c*vector->c - q_vector->d*vector->d; q_vector_q_star->b =
q_vector->a*vector->b + q_vector->b*vector->a + q_vector->c*vector->d -
q_vector->d*vector->c; q_vector_q_star->c = q_vector->a*vector->c -
q_vector->b*vector->d + q_vector->c*vector->a + q_vector->d*vector->b;
    q_vector_q_star->d = q_vector->a*vector->d + q_vector->b*vector->c -
q_vector->c*vector->b + q_vector->d*vector->a;

    return q_vector_q_star;

}

Point * world_to_body(float theta, Point * reference_point, float px, float py)
{
    // main

    // translation to match origins

    // https://www.youtube.com/watch?v=B9jDCj581Os

    Point * body;
    float rotation_matrix[[]] = [[cos(theta), sin(theta), px], [-sin(theta),
cos(theta), py], [0, 0, 1]]; body = rotation_matrix*reference_point; return
body;



}**/
void main() { return; }