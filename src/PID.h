#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Max Cross Tracking Error
  */
  double max_cte_;

  /*
  * Mean Square Error
  */
  int iter_;
  double sse_;
  double mse_;

  /*
  * Best MSE and PID Gains for Tuning
  */
  double best_mse_;
  double best_Kp_;
  double best_Ki_;
  double best_Kd_;
  int epoch_;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double lb, double ub);

  /*
  * Tune the PID gains by gridding the design space.
  */
  bool TunePID(std::vector<double> Kp_array, std::vector<double> Zd_array, std::vector<double> Ki_array);

};

#endif /* PID_H */
