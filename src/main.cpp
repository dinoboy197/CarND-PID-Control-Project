#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>


#include <algorithm>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}



void print_parameters(double parameters[]) {
  std::cout << "[" << parameters[0] << "," << parameters[1] << "," << parameters[2] << "]";
}

int main()
{
  uWS::Hub h;

  double parameters[] = {0.0969481,0.00625112,0.857888};
  double d_parameters[] = {0.0172907, 0.000321522, 0.0643044};

  PID pid;
  pid.init(parameters, true);
  double best_steering = 80.819;

  int current_p_index = 0;
  const int steps_per_evaluation = 1000;
  long total_steps = 0;
  int steps = 0;
  bool second_phase = false;
  double last_best_parameters[3];
  std::copy(std::begin(parameters), std::end(parameters), std::begin(last_best_parameters));
  std::cout << "saved parameters: ";
  print_parameters(last_best_parameters);
  std::cout << std::endl;
  std::cout << "d parameters: ";
  print_parameters(d_parameters);
  std::cout << std::endl;
  bool safety = false;
  bool had_to_engage_safety_mode = false;
  const double safety_limit = 2.5;

  h.onMessage([&pid, &steps, &second_phase, &best_steering, &d_parameters, &current_p_index, &parameters, &total_steps, &safety, &safety_limit, &last_best_parameters, &had_to_engage_safety_mode]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          if (cte > safety_limit) {
            // might run off track; makes training difficult as the simulation needs to be reset frequently
            // temporary reset parameters to last best until cte goes below 3
            pid.init(last_best_parameters, false);
            safety = true;
            had_to_engage_safety_mode = true;
            std::cout << "off-road safety mode engaged: ";
            print_parameters(last_best_parameters);
            std::cout << std::endl;
          } else if (safety && cte < safety_limit) {
            // cte is back to normal; set the parameters back to normal
            pid.init(parameters, false);
            safety = false;
            std::cout << "off-road safety mode disengaged";
            print_parameters(parameters);
            std::cout << std::endl;
          }

          double steer_value = pid.computeError(cte);

          ++total_steps;
          steps = (steps + 1) % steps_per_evaluation;
          if (steps == 0) {
            // if we're in a safety mode, reset back to non-safety mode
            if (safety) {
              safety = false;
              std::cout << "off-road safety mode disengaged" << std::endl;
              pid.init(parameters, false);
            }

            std::cout << "total steps " << total_steps << std::endl;
            double error = pid.getAndResetTotalError();
            if (!second_phase) {
              std::cout << "index " << current_p_index << " eval phase 1" << std::endl;
            }
            if (!second_phase && error < best_steering && !had_to_engage_safety_mode) {
              std::cout << "Error " << error << " better than best error " << best_steering << std::endl;
              std::copy(std::begin(parameters), std::end(parameters), std::begin(last_best_parameters));
              std::cout << "best parameters updated to: ";
              print_parameters(last_best_parameters);
              std::cout << std::endl;
              best_steering = error;

              d_parameters[current_p_index] *= 1.1;
              std::cout << "Updating d_parameters[" << current_p_index << "] to " << d_parameters[current_p_index] << std::endl;

              // move on to next parameter index to optimize
              current_p_index = (current_p_index + 1) % 3;
              parameters[current_p_index] += d_parameters[current_p_index];
              std::cout << std::endl << "Updating parameters[" << current_p_index << "] to " << parameters[current_p_index] << std::endl;

            } else {
              if (!second_phase) {
                std::cout << "Error " << error << " NOT better than best error " << best_steering << std::endl;
                parameters[current_p_index] -= 2 * d_parameters[current_p_index];
                parameters[current_p_index] = std::max(0.0, parameters[current_p_index]);
                std::cout << "Updating parameters[" << current_p_index << "] to " << parameters[current_p_index] << std::endl << "; ";
                print_parameters(parameters);
                std::cout << std::endl;
                pid.init(parameters, true);
                had_to_engage_safety_mode = false;
                std::cout << "Moving to phase 2..." << std::endl;
                // after another steps_per_evaluation steps, re-enter else clause directly
                second_phase = true;
              } else {
                std::cout << "index " << current_p_index << " eval phase 2" << std::endl;
                if (error < best_steering && !had_to_engage_safety_mode) {
                  std::cout << "Error " << error << " better than best error " << best_steering << std::endl;
                  std::copy(std::begin(parameters), std::end(parameters), std::begin(last_best_parameters));
                  std::cout << "best parameters updated to: ";
                  print_parameters(last_best_parameters);
                  std::cout << std::endl;
                  best_steering = error;

                  d_parameters[current_p_index] *= 1.1;
                  std::cout << "Updating d_parameters[" << current_p_index<< "] to " << d_parameters[current_p_index] << std::endl;
                } else {
                  std::cout << "Error " << error << " NOT better than best error " << best_steering << std::endl;
                  parameters[current_p_index] += d_parameters[current_p_index];
                  parameters[current_p_index] = std::max(0.0, parameters[current_p_index]);
                  std::cout << "Updating parameters[" << current_p_index << "] to " << parameters[current_p_index] << std::endl;
                  pid.init(parameters, true);
                  had_to_engage_safety_mode = false;
                  d_parameters[current_p_index] *= 0.9;
                  std::cout << "Updating d_parameters[" << current_p_index << "] to " << d_parameters[current_p_index] << std::endl;
                }

                // next evaluation phase should begin at the top
                second_phase = false;

                current_p_index = (current_p_index + 1) % 3;

                parameters[current_p_index] += d_parameters[current_p_index];
                std::cout << std::endl << "Updating parameters[" << current_p_index << "] to " << parameters[current_p_index] << std::endl;
              }
            }
          }

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
