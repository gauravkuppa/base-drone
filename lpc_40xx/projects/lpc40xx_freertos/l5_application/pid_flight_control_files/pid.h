typedef struct {

  float k_p, k_i, k_d;

  double last_time;
  float input;      // Leo: added
  float last_error; // Leo: Added
  float lastInput;
  float err_sum;
  float sampleTime;
  float last_filtered_error;
  float alpha;

  float expected_state, measured_state; // setpoint, input and output

  float motorRPMHighThreshold;
  float motorRPMLowThreshold;

} PID;

void PID_init(PID *pid, float motorRPMHighThreshold,
              float motorRPMLowThreshold);
void computePID(PID *pid, float expected_state, float measured_state);
void setTunings(PID *pid, float input_k_p, float input_k_i, float input_k_d);
void setSampleTime(PID *pid, float NewSampleTime);
void setMode(int Mode);