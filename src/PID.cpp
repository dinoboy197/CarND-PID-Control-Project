#include "PID.h"

#include <algorithm>

void PID::init(const double parameters[], const bool reset_errors) {
  this->Kp = parameters[0];
  this->Ki = parameters[1];
  this->Kd = parameters[2];
  if (reset_errors) {
    this->errors = std::vector<double>();
    this->current_cross_track_error = 0;
    this->previous_cross_track_error = 0;
    this->total_cross_track_error = 0;
  }
}

void PID::get_parameters(double new_parameters[]) {
  new_parameters[0] = Kp;
  new_parameters[1] = Ki;
  new_parameters[2] = Kd;
}

double PID::compute_error(double cte) {
  total_cross_track_error += cte;
  previous_cross_track_error = current_cross_track_error;
  current_cross_track_error = cte;

  double error = std::max(-1.0, std::min(1.0, -1 * current_cross_track_error * Kp - (current_cross_track_error - previous_cross_track_error) * Kd - total_cross_track_error * Ki));
  errors.push_back(std::abs(error));
  return error;
}

double PID::get_and_reset_total_error() {
  current_cross_track_error = 0;
  previous_cross_track_error = 0;
  total_cross_track_error = 0;
  double total_error = std::accumulate(errors.begin(), errors.end(), 0.0);
  errors.clear();
  return total_error;
}
