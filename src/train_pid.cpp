#include "train_pid.h"

#include <cmath>
#include <iostream>
#include <limits>

TrainPID::TrainPID(PID *pid, const long max_steps_per_evaluation, const double safety_limit, const bool training)
    : pid_(pid),
      kMaxStepsPerEvaluation(max_steps_per_evaluation),
      kSafetyLimit(safety_limit),
      kTraining(training) {
  safety_ = false;
  current_p_index_ = 0;
  had_to_engage_safety_mode_ = 0;
  steps_ = 0;
  total_steps_ = training ? 10.0 : kMaxStepsPerEvaluation;
  second_phase_ = false;
  best_error_ = std::numeric_limits<double>::max();

  pid->get_parameters(parameters_);
  pid->get_parameters(last_best_parameters_);
  // delta parameters are 10% of current parameter values
  copy_and_scale_parameters(parameters_, d_parameters_, 10.0);
}

void TrainPID::print_parameters(const double parameters[]) {
  std::cout << "[" << parameters[0] << "," << parameters[1] << "," << parameters[2] << "]";
}

void TrainPID::copy_and_scale_parameters(const double source[], double destination[], double divisor) {
  destination[0] = source[0] / divisor;
  destination[1] = source[1] / divisor;
  destination[2] = source[2] / divisor;
}

void TrainPID::increase_training_loop_size() {
  // if we're in training, slowly increase the time steps per training loop to ensure that fatal adjustments to
  // parameters only affect a the vehicle's performance for a few time steps before being re-evaluated
  if ((int) (total_steps_) < kMaxStepsPerEvaluation) {
    // training loop increases 10% per loop, up to the max
    total_steps_ *= 1.1;
    if ((int) (total_steps_) >= kMaxStepsPerEvaluation) {
      // when the training loop size has reached the max loop size, reset the best error rate to restart coordinate
      // ascent with the current parameters
      total_steps_ = kMaxStepsPerEvaluation;
      best_error_ = std::numeric_limits<double>::max();
    }
    std::cout << "total steps increased to " << total_steps_ << std::endl;
  }
  if ((int) (total_steps_) < kMaxStepsPerEvaluation && best_error_ < std::numeric_limits<double>::max()) {
    // when training loop size is still increasing, increase the best error rate 20% per loop to prevent small
    // segments of track with low error from dominating scale up times
    best_error_ *= 1.2;
    std::cout << "best error increased to " << best_error_ << std::endl;
  }
  // if we're in a safety mode, reset back to non-safety mode
  if (safety_) {
    safety_ = false;
    std::cout << "off-road safety mode disengaged" << std::endl;
    pid_->init(parameters_, false);
  }
}

void TrainPID::perform_training_adjustments(const double cross_track_error) {
  steps_ = (steps_ + 1) % (int) total_steps_;
  // when we're at the beginning of a training loop, adjust parameters with coordinate ascent algorithm
  if (steps_ == 0) {
    if (kTraining) {
      // if we're in training, slowly increase the time steps per training loop to ensure that fatal adjustments to
      // parameters only affect a the vehicle's performance for a few time steps before being re-evaluated
      increase_training_loop_size();
    }

    // get total error for the loop
    double error = pid_->get_and_reset_total_error();

    if (!kTraining) {
      // do not perform coordinate ascent if we're not in training
      return;
    } else {  
      std::cout << "total error on loop: " << error << std::endl;
    }

    if (!second_phase_) {
      std::cout << "index " << current_p_index_ << " eval phase 1" << std::endl;
    }
    if (!second_phase_ && error < best_error_ && !had_to_engage_safety_mode_) {
      // if we're in the first phase of testing for this parameter, and loop error is better than total error,
      // and we didn't engage safety mode during the loop,
      // save these parameters and adjust parameters and parameter deltas
      std::cout << "Error " << error << " better than best error " << best_error_ << std::endl;
      copy_and_scale_parameters(parameters_, last_best_parameters_);
      std::cout << "best parameters updated to: ";
      print_parameters(last_best_parameters_);
      std::cout << std::endl;
      best_error_ = error;

      d_parameters_[current_p_index_] *= 1.1;
      std::cout << "Updating d_parameters[" << current_p_index_ << "] to " << d_parameters_[current_p_index_]
                << std::endl;

      // move on to next parameter index to optimize
      current_p_index_ = (current_p_index_ + 1) % 3;
      parameters_[current_p_index_] += d_parameters_[current_p_index_];
      std::cout << std::endl << "Updating parameters[" << current_p_index_ << "] to " << parameters_[current_p_index_]
                << std::endl;

    } else {
      if (!second_phase_) {
        // if we're in the first phase of testing for this parameter, and we have not achieved better
        // performance, modify the parameters again, then the move onto the second phase
        std::cout << "Error " << error << " NOT better than best error " << best_error_ << std::endl;
        parameters_[current_p_index_] -= 2 * d_parameters_[current_p_index_];
        parameters_[current_p_index_] = std::max(0.0, parameters_[current_p_index_]);
        std::cout << "Updating parameters[" << current_p_index_ << "] to " << parameters_[current_p_index_] << std::endl
                  << "; ";
        print_parameters(parameters_);
        std::cout << std::endl;
        pid_->init(parameters_, true);
        had_to_engage_safety_mode_ = false;
        std::cout << "Moving to phase 2..." << std::endl;
        // after another steps_per_evaluation steps, re-enter else clause directly
        second_phase_ = true;
      } else {
        std::cout << "index " << current_p_index_ << " eval phase 2" << std::endl;
        if (error < best_error_ && !had_to_engage_safety_mode_) {
          // if we're in the second phase of testing for this parameter, and we have achieved better
          // performance and safety mode did not have to be engaged, saved the parameters as the best,
          // then change parameter deltas
          std::cout << "Error " << error << " better than best error " << best_error_ << std::endl;
          copy_and_scale_parameters(parameters_, last_best_parameters_);
          std::cout << "best parameters updated to: ";
          print_parameters(last_best_parameters_);
          std::cout << std::endl;
          best_error_ = error;

          d_parameters_[current_p_index_] *= 1.1;
          std::cout << "Updating d_parameters[" << current_p_index_ << "] to " << d_parameters_[current_p_index_]
                    << std::endl;
        } else {
          // if we're in the second phase of testing for this parameter, and we still have not achieved better
          // performance or safety mode had to be engaged, change parameters and parameter deltas
          std::cout << "Error " << error << " NOT better than best error " << best_error_ << std::endl;
          parameters_[current_p_index_] += d_parameters_[current_p_index_];
          parameters_[current_p_index_] = std::max(0.0, parameters_[current_p_index_]);
          std::cout << "Updating parameters[" << current_p_index_ << "] to " << parameters_[current_p_index_]
                    << std::endl;
          pid_->init(parameters_, true);
          had_to_engage_safety_mode_ = false;
          d_parameters_[current_p_index_] *= 0.9;
          std::cout << "Updating d_parameters[" << current_p_index_ << "] to " << d_parameters_[current_p_index_]
                    << std::endl;
        }

        // go back to phase 1
        second_phase_ = false;

        current_p_index_ = (current_p_index_ + 1) % 3;

        parameters_[current_p_index_] += d_parameters_[current_p_index_];
        std::cout << std::endl << "Updating parameters[" << current_p_index_ << "] to " << parameters_[current_p_index_]
                  << std::endl;
      }
    }
  }
}

void TrainPID::check_safety_mode(const double cross_track_error) {
  if (std::abs(cross_track_error) > kSafetyLimit) {
    // might run off track; makes training difficult as the simulation needs to be reset frequently
    // temporary reset parameters to last best until cross track error goes below maximum safe value
    pid_->init(last_best_parameters_, false);
    safety_ = true;
    had_to_engage_safety_mode_ = true;
    std::cout << "off-road safety mode engaged: ";
    print_parameters(last_best_parameters_);
    std::cout << std::endl;
  } else if (safety_ && std::abs(cross_track_error) < kSafetyLimit) {
    // cross track is within normal limits; set the parameters back to normal
    pid_->init(parameters_, false);
    safety_ = false;
    std::cout << "off-road safety mode disengaged";
    print_parameters(parameters_);
    std::cout << std::endl;
  }
}
