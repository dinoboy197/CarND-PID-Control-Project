#ifndef TRAIN_PID_H
#define TRAIN_PID_H

#include "PID.h"

class TrainPID {
 private:
  PID *pid_;
  const int kMaxStepsPerEvaluation;
  const double kSafetyLimit;
  const bool kTraining;

  double total_steps_;
  long steps_;
  double best_error_;
  bool safety_;
  bool had_to_engage_safety_mode_;
  int current_p_index_;
  bool second_phase_;
  double parameters_[3];
  double last_best_parameters_[3];
  double d_parameters_[3];

  void copy_and_scale_parameters(const double source[], double destination[], double divisor = 1.0);
  void print_parameters(const double parameters[]);
  void increase_training_loop_size();

 public:
  TrainPID(PID *pid, const long max_steps_per_evaluation, const double safety_limit, const bool training);

  void perform_training_adjustments(const double cross_track_error);
  void check_safety_mode(const double cross_track_error);
};

#endif /* TRAIN_PID_H */
