#include "pid.h"
#include <math.h>

#define M_PI 3.14159265358979323846

typedef struct {
  float w;
  float x;
  float y;
  float z;
} quaternion;

typedef struct {
  float x, y, z;
  float roll, pitch, yaw;
  float x_vel, y_vel, z_vel;
  float d_roll, d_pitch, d_yaw;
  float x_acc, y_acc, z_acc;
} state_space;

float * motor_mixing_algo(float thrust, float roll, float pitch, float yaw);

float thrust_pid(float reference_altitude, float estimated_altitude, float input_k_p, float input_k_i, float input_k_d);

float roll_pid(float reference_roll, float estimated_roll, float input_k_p, float input_k_i, float input_k_d);

float pitch_pid(float reference_pitch, float estimated_pitch, float input_k_p, float input_k_i, float input_k_d);

float yaw_pid(float reference_yaw, float estimated_yaw, float input_k_p, float input_k_i, float input_k_d);

float *body_to_world_quaternion(float *reference, float *euler, float *p);

float *body_to_world_3d_rotation(float *reference, quaternion *q);

quaternion *quaternion_multiply(quaternion *q1, quaternion *q0);

quaternion *euler_angle_to_quaternion(float euler_angle, int *rotational_axis);