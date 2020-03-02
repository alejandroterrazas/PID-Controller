#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;
}

void PID::UpdateError(double cte) { 
    d_error = cte - p_error;
   // std::cout << "d_error: " << d_error << std::endl;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
  double total_error = - Kp * p_error - Ki * i_error - Kd * d_error;
  return total_error;
}

