

//https://github.com/tekdemo/MiniPID/blob/master/MiniPID.cpp


struct S_PID{
    float k_p;
    float k_i;
    float k_d;

    float p;
    float i;
    float d;
};


// clamping, back calculation, observer approach
void updatePid(S_PID *pid, float estimated-state) {
    
    float motorRPMHighThreshold, motorRPMLowThreshold; // set these values to a little lower than actual motor RPM values
    bool saturated = false;
    bool allowIntegration = true;

    float error = 0; // desired_state - estimated_state; 
    pid->p = pid->k_p * error;
    pid->i = pid->k_i * integral(error); // how do i do an integral in C? best way?
    pid->d = pid->k_d * derivative(error); // how do i do a derivative in C, best way? efficiency?

    float newState = pid->p + pid->i + pid->d;

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
/*
    Create a Laplace transfer Function
*/



void laplaceDomain(S_PID *pid, float estimated_state){

    float timeDomainValue;
    float error = 0, s = 1; // desired_state - estimated_state; 
    pid->p = pid->k_p * error;
    pid->i = pid->k_i * integral(error); // how do i do an integral in C? best way?
    pid->d = pid->k_d * derivative(error); 

    timeDomainValue =  pid->p + pid->i + pid->d;
    float oldDeriv = 
}