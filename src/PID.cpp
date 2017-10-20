#include "PID.h"

#include <numeric>
using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    error_p = 0.0;
    error_i = 0.0;
    error_d = 0.0;
    first = true;

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    square_error = 0.0;
}

double PID::Control(double error) {

    square_error += (error*error);

    error_d = 0.0;
    if(!first) {
        error_d = error - error_p;
    } else {
        first = false;
    }
    error_p = error;
    error_i += error;
    return -(Kp * error_p + Kd * error_d + Ki * error_i);
}

double PID::TotalError() {
    return square_error;
}

