#include "PID.h"

#include <algorithm>

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->current_cross_track_error = 0;
  this->previous_cross_track_error = 0;
}

void PID::UpdateError(double cte) {
  total_cross_track_error += cte;
  previous_cross_track_error = current_cross_track_error;
  current_cross_track_error = cte;
}

double PID::TotalError() {
  return std::max(-1.0, std::min(1.0, -1 * current_cross_track_error * Kp - (current_cross_track_error - previous_cross_track_error) * Kd - total_cross_track_error * Ki));
}
