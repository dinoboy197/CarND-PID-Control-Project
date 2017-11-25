#ifndef PID_H
#define PID_H

#include <vector>

class PID {
 private:

  double current_cross_track_error;
  double previous_cross_track_error;
  double total_cross_track_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  std::vector<double> errors;

 public:
  /*
  * Initialize PID parameters.
  */
  void init(const double parameters[], const bool reset_errors);

  void get_parameters(double parameters[]);

  /*
  * Update the PID error variables given cross track error and return the error.
  */
  double computeError(double cte);

  /*
  * Calculate reset, and return the total PID error.
  */
  double getAndResetTotalError();
};

#endif /* PID_H */
