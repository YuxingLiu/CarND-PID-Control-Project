#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
}

void PID::UpdateError(double cte) {
  i_error_ += cte;
  d_error_ = cte - p_error_;
  p_error_ = cte;
}

double PID::TotalError() {
  double steer = - Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;

  // steering value is in [-1, 1]
  if(steer > 1)
    steer = 1;
  else if (steer < -1)
    steer = -1;

  return steer;
}

