// clamping, back calculation, observer approach
void pid(float estimated-state) {
    
    float motorRPMHighThreshold, motorRPMLowThreshold; // set these values to a little lower than actual motor RPM values
    bool saturated = false;
    bool allowIntegration = true;
    float k_p;
    float k_i;
    float k_d;

    float error = 0; // desired_state - estimated_state; 
    float p = k_p * error;
    float i = k_i * integral(error); // how do i do an integral in C? best way?
    float d = k_d * derivative(error); // how do i do a derivative in C, best way? efficiency?

    float newState = p + i + d;

    // clamping for anti-windup integration

    if (newState > motorRPMHighThreshold) {
        newState = motorRPMHighThreshold;
        if (error > 0) {
            allowIntegration = false;
        }
    } else if (newState < motorRPMLowThreshold) {
        newState = motorRPMLowThreshold;
        if (error < 0) {
            allowIntegration = false;
        } 
    }


    // TODO: if not allowIntegration, then what?


    // derivative path enlarges values of noise
    // derivative --> low pass filter
    // use laplace transform function??


    




}
