#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double p[3]    = {0.0, 0.0, 0.0} ;
bool twiddling = true;

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


double steering_smoother(double pre_smooth_steer)
{
    double new_steer = pre_smooth_steer;
    if (new_steer > 1.0) {
        new_steer = 1.0;
    }
    if (new_steer < -1.0) {
        new_steer = -1.0;
    }

    return new_steer;
    
}



int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
    p[0] = 0.239388;//0.25; // Kp
    p[1] = 0.000107181; //0.0001 Ki
    p[2] = 5.13356; //4.0; // Kd

    pid.Init(p[0], p[1], p[2]);
    pid.is_twiddling = false; // comment this line for twiddling
    //int iter_count = 0;
    
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
      pid.iteration_for_twiddle ++;
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
          double steer_value;
            
          pid.UpdateError(cte);

          double pre_smooth_steer = pid.TotalError();
            
          steer_value = steering_smoother(pre_smooth_steer) ;

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
      // Twiddle
      if (pid.iteration_for_twiddle % 2000 == 0) {
          std::cout << "heart beat:" << pid.iteration_for_twiddle << std::endl;
      }
      if (pid.should_restart_twiddle_iteration()) { // restart count define 1 iteration
          std::cout << "iteration: " << pid.iteration_for_twiddle / 2000 << std::endl;
          std::cout << "total error: " << pid.total_error << std::endl;
          std::cout << "best error: " << pid.best_error << std::endl;
          std::cout << "PID: " << pid.Kp << "," << pid.Ki << "," <<pid.Kd<< std::endl;
          pid.twiddle();
          // restart
          pid.i_error = 0.0;
          std::string reset_msg = "42[\"reset\",{}]";
          ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
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
