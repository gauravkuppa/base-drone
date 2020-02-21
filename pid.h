class PID{
    public:
        float k_p, k_i, k_d;

        double last_time;
        float last_error;
        float err_sum;
        float last_filtered_error = NULL;
        float alpha = 0.4;

        float expected_state, measured_state; // setpoint, input and output

        float motorRPMHighThreshold;
        float motorRPMLowThreshold;

        PID(float motorRPMHighThreshold, float motorRPMLowThreshold);
        void tuneParameters(float input_k_p, float input_k_i, float input_k_d);
        void computePID(float expected_state, float measured_state);
};