#include "PID.h"

PID::PID(float Kp, float Kd, float Ki, float target) {
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    _target = target;
}

float PID::getControl(float value, float lastValue, float dt) {
    float error = _target - value;
    float de = -(value - lastValue) / dt;
    _integralError += _Ki * error * dt;
    _lastError = error;
    return (_Kp * error + _Kd * de + _integralError);
}

void PID::setTarget(float target) {
    _target = target;
}