#include "PID.h"

#include <cmath>
#include <numeric>

PID::PID() {
  errors_ = std::vector<double>();
  current_cross_track_error_ = 0;
  previous_cross_track_error_ = 0;
  total_cross_track_error_ = 0;
  Kd_ = 0;
  Ki_ = 0;
  Kp_ = 0;
}

void PID::init(const double parameters[], const bool reset_errors) {
  Kp_ = parameters[0];
  Ki_ = parameters[1];
  Kd_ = parameters[2];
  if (reset_errors) {
    errors_ = std::vector<double>();
    current_cross_track_error_ = 0;
    previous_cross_track_error_ = 0;
    total_cross_track_error_ = 0;
  }
}

void PID::get_parameters(double new_parameters[]) {
  new_parameters[0] = Kp_;
  new_parameters[1] = Ki_;
  new_parameters[2] = Kd_;
}

double PID::compute_steering_angle(const double cross_track_error) {
  total_cross_track_error_ += cross_track_error;
  previous_cross_track_error_ = current_cross_track_error_;
  current_cross_track_error_ = cross_track_error;
  errors_.push_back(std::abs(cross_track_error));

  // return steering control, limited between [-1,1], then normalized by fraction of current speed to max speed
  return std::max(-1.0, std::min(1.0,
     - current_cross_track_error_ * Kp_
     - (current_cross_track_error_ - previous_cross_track_error_) * Kd_
     - total_cross_track_error_ * Ki_));
}

double PID::get_and_reset_total_error() {
  current_cross_track_error_ = 0;
  previous_cross_track_error_ = 0;
  total_cross_track_error_ = 0;
  double total_error = std::accumulate(errors_.begin(), errors_.end(), 0.0);
  errors_.clear();
  return total_error;
}
