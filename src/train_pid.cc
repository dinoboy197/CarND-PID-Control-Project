#include "train_pid.h"

#include <algorithm>
#include <iostream>

#include "train_pid.h"

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
  best_steering_ = std::numeric_limits<double>::max();

  pid->get_parameters(parameters_);
  pid->get_parameters(last_best_parameters_);
  copy_parameters(parameters_, d_parameters_, 10.0);
}

void TrainPID::print_parameters(const double parameters[]) {
  std::cout << "[" << parameters[0] << "," << parameters[1] << "," << parameters[2] << "]";
}

void TrainPID::copy_parameters(const double source[], double destination[], double divisor) {
  destination[0] = source[0] / divisor;
  destination[1] = source[1] / divisor;
  destination[2] = source[2] / divisor;
}

void TrainPID::perform_training_adjustments(const double cross_track_error) {

  steps_ = (steps_ + 1) % (int) total_steps_;
  if (steps_ == 0) {
    if (kTraining) {
      if ((int) total_steps_ < kMaxStepsPerEvaluation) {
        total_steps_ *= 1.1;
        if ((int) total_steps_ >= kMaxStepsPerEvaluation) {
          best_steering_ = std::numeric_limits<double>::max();
        }
        std::cout << "total steps increased to " << total_steps_ << std::endl;
      }
      if ((int) total_steps_ < kMaxStepsPerEvaluation && best_steering_ < std::numeric_limits<double>::max()) {
        best_steering_ *= 1.2;
        std::cout << "best error increased to " << best_steering_ << std::endl;
      }

      // if we're in a safety mode, reset back to non-safety mode
      if (safety_) {
        safety_ = false;
        std::cout << "off-road safety mode disengaged" << std::endl;
        pid_->init(parameters_, false);
      }
    }

    double error = pid_->get_and_reset_total_error();

    if (!kTraining) {
      std::cout << "total error on loop: " << error << std::endl;
      return;
    }

    if (!second_phase_) {
      std::cout << "index " << current_p_index_ << " eval phase 1" << std::endl;
    }
    if (!second_phase_ && error < best_steering_ && !had_to_engage_safety_mode_) {
      std::cout << "Error " << error << " better than best error " << best_steering_ << std::endl;
      copy_parameters(parameters_, last_best_parameters_);
      std::cout << "best parameters updated to: ";
      print_parameters(last_best_parameters_);
      std::cout << std::endl;
      best_steering_ = error;

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
        std::cout << "Error " << error << " NOT better than best error " << best_steering_ << std::endl;
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
        if (error < best_steering_ && !had_to_engage_safety_mode_) {
          std::cout << "Error " << error << " better than best error " << best_steering_ << std::endl;
          copy_parameters(parameters_, last_best_parameters_);
          std::cout << "best parameters updated to: ";
          print_parameters(last_best_parameters_);
          std::cout << std::endl;
          best_steering_ = error;

          d_parameters_[current_p_index_] *= 1.1;
          std::cout << "Updating d_parameters[" << current_p_index_ << "] to " << d_parameters_[current_p_index_]
                    << std::endl;
        } else {
          std::cout << "Error " << error << " NOT better than best error " << best_steering_ << std::endl;
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

        // next evaluation phase should begin at the top
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
    // temporary reset parameters to last best until cte goes below 3
    pid_->init(last_best_parameters_, false);
    safety_ = true;
    had_to_engage_safety_mode_ = true;
    std::cout << "off-road safety mode engaged: ";
    print_parameters(last_best_parameters_);
    std::cout << std::endl;
  } else if (safety_ && std::abs(cross_track_error) < kSafetyLimit) {
    // cte is back to normal; set the parameters back to normal
    pid_->init(parameters_, false);
    safety_ = false;
    std::cout << "off-road safety mode disengaged";
    print_parameters(parameters_);
    std::cout << std::endl;
  }
}
