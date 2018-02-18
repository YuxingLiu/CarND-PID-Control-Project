#ifndef PID_H
#define PID_H

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
  * Actuator Constraints
  */
  double lb_;
  double ub_;

  /*
  * Max Cross Tracking Error
  */
  double max_cte_;

  /*
  * Sum of Square Error
  */
  int iter_;
  double sse_;

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
  void Init(double Kp, double Ki, double Kd, double lb, double ub);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate the root mean square error
  */
  double RootMeanSquareError();
};

#endif /* PID_H */
