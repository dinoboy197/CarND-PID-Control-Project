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

int main()
{
  uWS::Hub h;

  double parameters[] = {0.125, 0.0, 0.8};
  double d_parameters[] = {0.0,0.0,0.0}; //{0.2, 0.000001, 1};

  PID pid;
  pid.init(parameters);
  double best_steering = std::numeric_limits<double>::max();

  int current_p_index = 0;
  const int steps_per_evaluation = 10;
  int steps = 0;
  bool second_phase = false;

  h.onMessage([&pid, &steps, &second_phase, &best_steering, &d_parameters, &current_p_index, &parameters](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          double steer_value = pid.computeError(cte);

          if (speed < 20) {
            pid.getAndResetTotalError();
          } else {
            steps = (steps + 1) % steps_per_evaluation;
            if (steps == 0) {

              double error = pid.getAndResetTotalError();
              if (!second_phase) {
                std::cout << "index " << current_p_index << " eval phase 1" << std::endl;
              }
              if (!second_phase && error < best_steering) {
                std::cout << "Error " << error << " better than best error " << best_steering << std::endl;
                best_steering = error;
                d_parameters[current_p_index] *= 1.1;
                std::cout << "Updating d_parameters[" << current_p_index << "] to " << d_parameters[current_p_index] << std::endl;

                // move on to next parameter index to optimize
                if (current_p_index == 0) {
                  // skip i parameter;
                  current_p_index += 1;
                }

                // move on to next parameter index to optimize
                current_p_index = (current_p_index + 1) % 3;

                std::cout << std::endl << "Updating parameters[" << current_p_index << "] to " << parameters[current_p_index] << std::endl;
                parameters[current_p_index] += d_parameters[current_p_index];
              } else {
                if (!second_phase) {
                  std::cout << "Error " << error << " NOT better than best error " << best_steering << std::endl;
                  parameters[current_p_index] -= 2 * d_parameters[current_p_index];
                  parameters[current_p_index] = std::max(0.0, parameters[current_p_index]);
                  std::cout << "Updating parameters[" << current_p_index << "] to " << parameters[current_p_index] << std::endl;
                  pid.init(parameters);
                  std::cout << "Moving to phase 2..." << std::endl;
                  // after another steps_per_evaluation steps, re-enter else clause directly
                  second_phase = true;
                } else {
                  std::cout << "index " << current_p_index << " eval phase 2" << std::endl;
                  if (error < best_steering) {
                    std::cout << "Error " << error << " better than best error " << best_steering << std::endl;
                    best_steering = error;
                    d_parameters[current_p_index] *= 1.1;
                    std::cout << "Updating d_parameters[" << current_p_index<< "] to " << d_parameters[current_p_index] << std::endl;
                  } else {
                    std::cout << "Error " << error << " NOT better than best error " << best_steering << std::endl;
                    parameters[current_p_index] += d_parameters[current_p_index];
                    parameters[current_p_index] = std::max(0.0, parameters[current_p_index]);
                    std::cout << "Updating parameters[" << current_p_index << "] to " << parameters[current_p_index] << std::endl;
                    pid.init(parameters);
                    d_parameters[current_p_index] *= 0.9;
                    std::cout << "Updating d_parameters[" << current_p_index << "] to " << d_parameters[current_p_index] << std::endl;
                  }

                  // next evaluation phase should begin at the top
                  second_phase = false;

                  // move on to next parameter index to optimize
                  if (current_p_index == 0) {
                    // skip i parameter;
                    current_p_index += 1;
                  }
                  current_p_index = (current_p_index + 1) % 3;

                  std::cout << std::endl << "Updating parameters[" << current_p_index << "] to " << parameters[current_p_index] << std::endl;
                  parameters[current_p_index] += d_parameters[current_p_index];
                }
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
