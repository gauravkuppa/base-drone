struct S_PID{
    float k_p;
    float k_i;
    float k_d;

    float p;
    float i;
    float d;
};

PID::PID(double p, double i, double d){
    init();
    P = p; 
    I = i; 
    D = d;
}

PID::PID(double p, double i, double d, double f){
    init();
    P=p;
    I=i;
    D=d;
    F=f;
}

void PID::init(){
    P=0;
    I=0;
    D=0;
    F=0;

    maxIOutput=0;
    maxError=0;
    errorSum=0;
    maxOutput=0; 
    minOutput=0;
    setpoint=0;
    lastActual=0;
    firstRun=true;
    reversed=false;
    outputRampRate=0;
    lastOutput=0;
    outputFilter=0;
    setpointRange=0;
}

/*
@param p = proportional gain. 
*/
void PID::setP(double p){
    P=p;
    checkSigns();
}

void PID:: setI(double i){
    if(I!=0) errorSum = errorSum*I/i; 
    if(maxIOutput != 0) maxError = maxIOutput/i;
    I=i;
    checkSigns();
}

void PID::setD(double d){
    D=d;
    checkSigns();
}
//F is FeedForward parameter
void PID::setF(double f){
    F=f;
    checkSigns();
}

void PID::setPID(double p, double i, double d){
    P = p; 
    I = i;
    D = d;
    checkSigns();
}

void PID::setPID(double p, double i, double d, double f){
    P=p; I=i; D=d; F=f;
    CheckSigns();
}

void PID::SetMaxIOutput(double maximum){
    //maxerror and Izone are similar, but scalled for diff purposes
    //the maxError is generated for simplofying math. Cakculations against the max error
    //are far more common than changing the I term of Izone
    maxIOutput = maximum;
    if(I!=0) maxError = maxIOutput/I;
} 


void PID::setOutputLimits(double output){ //specify a max output
    setOutputLimits(-output, output);
}
void PID::setDirection(bool reversed){
    this->reversed = reversed;
}

void PID::setSetpoint(double setpoint){
    this->setpoint = setpoint;
}


double PID::getOutput(double actual, double setpoint){
    double output, Poutput, Ioutput, Doutput, Foutput;
    this->setpoint = setpoint;

    if(setpointRange != 0) setpoint = clamp(setpoint, actual-setpointRange, actual+setpointRange);
    double error = setpoint - actual;

    Poutput = P*error; //Calc the p term

    if(firstRun){
        lastActual = actual;
        lastOutput = Poutput + Foutput;
        firstRun = false;
    }

    //D term calculation
    Doutput = -D*(actual - lastActual);
    lastActual = actual;

    //Iterm. Complex steps. 
    // MaxIoutput restricts amnt of output contributed by Iterm
    //Prevent windup by not increasing errorSum if we're already running against the maxIoutput
    //Prevent windup by not incr errorSum if output is output=maxOutput
    Ioutput = I*errorSum;
    if(maxIoutput !=0) Ioutput = clamp(Ioutput,-maxIoutput,maxIOutput);
    
    //Output summation
    output  = Foutput + Poutput + Doutput;

    //error
    if(minOutput != maxOutput && !bounded(output, minOutput, maxOutput)) errorSum = error;
    else if(outputRampRate!=0 && !bounded(output, lastOutput - outputRampRate, lastOutput+outputRampRate)){
        errorSum = error;
    }
    else errorSum += error;

    //Restrict output to specified output and ramp limitations
    if(outputRampRate != 0) output = clamp(output, lastOutput - outputRampRate, lastOutput + outputRampRate);
    if(minOutput != maxOutput) output = clamp(output, minOutput, maxOutput);
    if(outputFilter !=0) output = lastOutput*outputFilter+output*(1 - outputFilter);

    lastOutput = output;
    return output;

}

double PID::getOutput(){
    return getOutput(lastActual, setpoint);
}

double PID::getOutput(double actual){
    return getOutput(actual,setpoint);
}

void PID::reset(){
    firstRun = true;
    errorSum = 0;
}

void PID::setSetpointRange(double range){
    setpointRange = range;
}

//Filter for sharp oscillations
void PID::setOutputFilter(double strength){
    if(strength == 0 || bounded(strength, 0, 1)) outputFilter = strength;
}


//Other Functions //
double PID::clamp(double value, double min, double max){
    if(value>max) return max;
    if(value < min) return min;
    return value;
}

//Test if the value is within the min and max, inclusive
bool PID::bounded(double value, double min, double max){
    return (min<value) && (value < max);
}

//PID parameters require the same sign
void PID::checkSigns(){
    if(reversed){
        if(P>0) P *= -1;
        if(I>0) I *= -1;
        if(D>0) D *= -1;
        if(F>0) F *= -1;
    }
    else{
        if(P<0) P *= -1;
        if(I<0) I *= -1;
        if(D<0) D *= -1;
        if(F<0) F *= -1;
    }

}

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