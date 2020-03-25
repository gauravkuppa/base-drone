
class PID{
    public:
        float k_p, k_i, k_d;

        double last_time;
        float input; //Leo: added
        float last_error; //Leo: Added
        float lastInput;
        float err_sum;
        float sampleTime = 1000/200000;  // sampling once per 5 microseconds
        float last_filtered_error = NULL;
        float alpha = 0.4; // the higher the value of alpha, the less depenecy the filtered_d has on older values

        float expected_state, measured_state; // setpoint, input and output

        float motorRPMHighThreshold;
        float motorRPMLowThreshold;

        PID(float motorRPMHighThreshold, float motorRPMLowThreshold);
        void computePID(float expected_state, float measured_state);
        void setTunings(float input_k_p, float input_k_i, float input_k_d);
        void setSampleTime(float NewSampleTime);
        void setMode(int Mode);
};