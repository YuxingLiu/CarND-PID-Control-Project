#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double lb, double ub) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  lb_ = lb;
  ub_ = ub;

  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;

  max_cte_ = 0.0;

  iter_ = 0;
  sse_ = 0.0;
}

void PID::UpdateError(double cte) {
  i_error_ += cte;
  d_error_ = cte - p_error_;
  p_error_ = cte;

  iter_ += 1;
  sse_ += cte * cte;

  if(max_cte_ < fabs(cte))
    max_cte_ = fabs(cte);
}

double PID::TotalError() {
  double total = Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;

  if(total > ub_)
    total = ub_;
  else if (total < lb_)
    total = lb_;

  return total;
}

double PID::RootMeanSquareError() {
  if(iter_ == 0)
    return 0.0;
  else
    return sse_ / iter_;
}

