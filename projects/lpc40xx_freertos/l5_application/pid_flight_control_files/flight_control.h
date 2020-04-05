struct Quaternion {
  float a;
  float b;
  float c;
  float d;
};

typedef struct {
  float x, y, z;
  float roll, pitch, yaw;
  float x_vel, y_vel, z_vel;
  float d_roll, d_pitch, d_yaw;
  float x_acc, y_acc, z_acc;
} state_space;