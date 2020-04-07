#include "flight_control.h"

// TODO: Motor
// TODO: create makefile

float * motor_mixing_algo(float thrust, float roll, float pitch, float yaw) {
  float motors[4];
  motors[0] = thrust + roll + pitch + yaw;
  motors[1] = thrust - roll + pitch - yaw;
  motors[2] = thrust + roll - pitch - yaw;
  motors[3] = thrust - roll - pitch + yaw;
  return motors;
}

float thrust_pid(float reference_altitude, float estimated_altitude, float input_k_p, float input_k_i, float input_k_d) {
  PID *pid;
  PID_init(pid, 0, 1000 * 14.7); // high RPM = kV * battery Voltage [a little
                                 // lower than actual value]
  setTunings(pid, input_k_p, input_k_i, input_k_d);
  computePID(pid, reference_altitude, estimated_altitude);
  return pid->measured_state;
}
float roll_pid(float reference_roll, float estimated_roll, float input_k_p, float input_k_i, float input_k_d) {
  PID *pid;
  PID_init(pid, 0, 1000 * 14.7); // high RPM = kV * battery Voltage [a little
                                 // lower than actual value]
  setTunings(pid, input_k_p, input_k_i, input_k_d);
  computePID(pid, reference_roll, estimated_roll);
  return pid->measured_state;
}

float pitch_pid(float reference_pitch, float estimated_pitch, float input_k_p, float input_k_i, float input_k_d) {
  PID *pid;
  PID_init(pid, 0, 1000 * 14.7); // high RPM = kV * battery Voltage [a little
                                 // lower than actual value]
  setTunings(pid, input_k_p, input_k_i, input_k_d);
  computePID(pid, reference_pitch, estimated_pitch);
  return pid->measured_state;
}

float yaw_pid(float reference_yaw, float estimated_yaw, float input_k_p, float input_k_i, float input_k_d) {
  PID *pid;
  PID_init(pid, 0, 1000 * 14.7); // high RPM = kV * battery Voltage [a little
                                 // lower than actual value]
  setTunings(pid, input_k_p, input_k_i, input_k_d);
  computePID(pid, reference_yaw, estimated_yaw);
  return pid->measured_state;
}

float *body_to_world_quaternion(float *reference, float *euler, float *p) {
  float px = p[0];
  float py = p[1];
  float pz = p[2];
  float roll = euler[0];
  float pitch = euler[1];
  float yaw = euler[2];
  int rotational_axis_roll[3] = {1, 0, 0};
  int rotational_axis_pitch[3] = {0, 1, 0};
  int rotational_axis_yaw[3] = {0, 0, 1};

  // remember to add 1 to the end of reference_point when passing it in
  float reference_point[4];
  reference_point[0] = reference[0];
  reference_point[1] = reference[1];
  reference_point[2] = reference[2];
  reference_point[3] = 1.0;

  quaternion *quaternion_yaw =
      euler_angle_to_quaternion(roll, &rotational_axis_yaw);
  quaternion *quaternion_pitch =
      euler_angle_to_quaternion(pitch, &rotational_axis_pitch);
  quaternion *quaternion_roll =
      euler_angle_to_quaternion(yaw, &rotational_axis_roll);

  quaternion *composed_yaw_pitch =
      quaternion_multiply(quaternion_yaw, quaternion_pitch);
  quaternion *composed_rotation_zyx =
      quaternion_multiply(composed_yaw_pitch, quaternion_roll);

  quaternion *reference_prime =
      body_to_world_3d_rotation(&reference_point, composed_rotation_zyx);

  reference_prime->x += px;
  reference_prime->y += py;
  reference_prime->z += pz;

  float *vector;
  vector[0] = reference_prime->x;
  vector[1] = reference_prime->y;
  vector[2] = reference_prime->z;
  return vector;
}

float *body_to_world_3d_rotation(float *reference, quaternion *q) {
  quaternion *q_star;
  quaternion *vector;

  q_star->w = q->w;
  q_star->x = -1 * q->x;
  q_star->y = -1 * q->y;
  q_star->z = -1 * q->z;

  vector->w = 0;
  vector->x = reference[0];
  vector->y = reference[1];
  vector->z = reference[2];

  quaternion *q_vector = quaternion_multiply(q, vector);
  quaternion *reference_prime = quaternion_multiply(q_vector, q_star);

  return reference_prime;
}

quaternion *quaternion_multiply(quaternion *q1, quaternion *q0) {
  float w0 = q0->w;
  float x0 = q0->x;
  float y0 = q0->y;
  float z0 = q0->z;
  float w1 = q1->w;
  float x1 = q1->x;
  float y1 = q1->y;
  float z1 = q1->z;

  quaternion *q0q1;
  q0q1->w = -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0;
  q0q1->x = x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0;
  q0q1->y = -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0;
  q0q1->z = x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0;

  return q0q1;
}

quaternion *euler_angle_to_quaternion(float euler_angle, int *rotational_axis) {
  float theta = euler_angle * (M_PI / 180);

  quaternion *q;
  q->w = (float)cos(theta / 2);
  q->x = (float)sin(theta / 2) * rotational_axis[0];
  q->y = (float)sin(theta / 2) * rotational_axis[1];
  q->z = (float)sin(theta / 2) * rotational_axis[2];

  return q;
}

void main() { return; }