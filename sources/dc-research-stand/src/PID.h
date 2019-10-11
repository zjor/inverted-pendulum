#ifndef PID_H
#define PID_H

class PID {
    public:
        PID(float Kp, float Kd, float Ki, float target);
        float getControl(float value, float lastValue, float dt);
        void setTarget(float target);
    private:
        float _Kp;
        float _Kd;
        float _Ki;
        float _lastError;
        float _integralError;
        float _target;
};

#endif