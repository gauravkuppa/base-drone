float k_p;
float k_i;
float k_d;

float error = 0; // desired_state - estimated_state; 
float p = k_p * error;
float i = k_i * integral(error); // how do i do an integral in C? best way?
float d = k_d * derivative(error); // how do i do a derivative in C, best way? efficiency?