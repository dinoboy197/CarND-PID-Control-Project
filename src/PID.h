#ifndef PID_H
#define PID_H

#include <vector>

class PID {
 private:

  // individual and total errors
  double current_cross_track_error;
  double previous_cross_track_error;
  double total_cross_track_error;
  std::vector<double> errors;

  // pid coefficients
  double Kp;
  double Ki;
  double Kd;

 public:
  /*
   * Initialize PID parameters.
   */
  void init(const double parameters[], const bool reset_errors);

  /*
   * Get parameters.
   */
  void get_parameters(double parameters[]);

  /*
   * Update the PID error variables given cross track error and return the error.
   */
  double compute_error(double cte);

  /*
   * Calculate reset, and return the total PID error.
   */
  double get_and_reset_total_error();
};

#endif /* PID_H */
