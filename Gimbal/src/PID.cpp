#include "PID.h"

PIDController::PIDController(float p, float i, float d) : kp(p), ki(i), kd(d), integral(0), previouserror(0) {

    kp = p;
    ki = i;
    kd = d;
    integral = 0;
    previouserror = 0;
    
}

float PIDController::compute(float setpoint, float measurement, float dt){

    float error = setpoint - measurement;

    integral += error*dt;
    float derivative = (error-previouserror)/dt;

    previouserror = error;
    return kp*error + ki*integral + kd*derivative;

}


void PIDController::reset(){
    integral = 0;
    previouserror = 0;
}