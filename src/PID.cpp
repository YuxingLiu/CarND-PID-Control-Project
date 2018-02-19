#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  best_mse_ = 1000;
  epoch_ = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;

  max_cte_ = 0.0;

  iter_ = 0;
  sse_ = 0.0;
  mse_ = 0.0;
}

void PID::UpdateError(double cte) {
  i_error_ += cte;
  d_error_ = cte - p_error_;
  p_error_ = cte;

  iter_ += 1;
  sse_ += cte * cte;
  mse_ = sse_ / iter_;

  if(max_cte_ < fabs(cte))
    max_cte_ = fabs(cte);
}

double PID::TotalError(double lb, double ub) {
  double total = Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;

  // Saturation due to actuator limits
  if(total > ub)
    total = ub;
  else if (total < lb)
    total = lb;

  return total;
}

bool PID::TunePID(std::vector<double> Kp_array, std::vector<double> Zd_array, std::vector<double> Ki_array) {
  int n_p = Kp_array.size();
  int n_d = Zd_array.size();
  int n_i = Ki_array.size();

  // Output current step info
  std::cout << std::fixed;
  std::cout << "Epoch: " << epoch_ << "\tKp: " << Kp_ << "\tKi: " << Ki_ << "\tKd: " << Kd_ << "\tMSE: " << mse_ << "\tMax cte: " << max_cte_ << "\tIter: " << iter_ << std::endl;
  std::cout << std::endl;

  // Update the best
  if(best_mse_ > mse_) {
    best_mse_ = mse_;
    best_Kp_ = Kp_;
    best_Ki_ = Ki_;
    best_Kd_ = Kd_;
  }

  // Update the PID gains
  if(epoch_ < n_p*n_d + n_i) {
    // PID is not yet tuned

    if(epoch_ < n_p*n_d){
      // Tune PD
      int i_p = epoch_ / n_d;
      int i_d = epoch_ % n_d;
      double Kp = Kp_array[i_p];
      double Kd = Kp_array[i_p] * Zd_array[i_d];
      double Ki = 0.0;
      Init(Kp, Ki, Kd);
    }else {
      // Tune I
      int i_i = epoch_ - n_p*n_d;
      double Kp = best_Kp_;
      double Kd = best_Kd_;
      double Ki = Ki_array[i_i];
      Init(Kp, Ki, Kd);
    }

    epoch_ += 1;
    return 0;
  }else {
    // PID is tuned

    std::cout << "PID is tuned.\tKp: " << best_Kp_ << "\tKi: " << best_Ki_ << "\tKd: " << best_Kd_ << "\tMSE: " << best_mse_ << std::endl;
    std::cout << std::endl;

    Init(best_Kp_, best_Ki_, best_Kd_);
    return 1;
  }

}

