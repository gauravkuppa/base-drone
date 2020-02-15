class PID{
    public:
        float k_p;
        float k_i;
        float k_d;

        float p;
        float i;
        float d;

        float current_state;
        float motorRPMHighThreshold;
        float motorRPMLowThreshold;

        PID(float motorRPMHighThreshold, float motorRPMLowThreshold);
        void tuneParameters(float input_k_p, float input_k_i, float input_k_d);
        void updatePID(float expected_state, float measured_state);
};