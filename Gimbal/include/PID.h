#ifndef PID_H
#define PID_H

class PIDController{
    private:
        float kp;
        float ki;
        float kd;

        float integral;
        float previouserror;


    public:
        PIDController(float p, float i, float d);
        float compute(float setpoint, float measurement, float dt);
        void reset();


};

#endif