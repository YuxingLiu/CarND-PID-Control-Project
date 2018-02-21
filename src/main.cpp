#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>

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

  PID pid_steering, pid_throttle;
  // Initialize the pid variable.
  pid_steering.Init(0.25, 1e-3, 7.5);
  pid_throttle.Init(0.1, 0.0001, 2.0);

  // Flag, if steering PID is tuned
  bool is_pid_tuned = 1;
  std::vector<double> Kp_array = {0.1, 0.15, 0.2, 0.25};
  std::vector<double> Zd_array = {20, 30, 40, 50};
  std::vector<double> Ki_array = {1e-5, 1e-4, 1e-3};

  h.onMessage([&pid_steering, &pid_throttle, &is_pid_tuned, &Kp_array, &Zd_array, &Ki_array](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());

	  // Steering control
	  pid_steering.UpdateError(-cte);
	  double steer_value = pid_steering.TotalError(-1.0, 1.0);

          // Throttle control;
          double speed_ref = 40.0;
          double error_speed = speed_ref - speed;
          pid_throttle.UpdateError(error_speed);
          double throttle_value = pid_throttle.TotalError(0.0, 1.0);

	  // Estimated steps for one drive cycle
          int iter_1cycle = 5800;
	  if(!is_pid_tuned && pid_steering.iter_ == iter_1cycle) {
	    // Steering PID tuning
	    is_pid_tuned = pid_steering.TunePID(Kp_array, Zd_array, Ki_array);
	  }else if(is_pid_tuned) {
	    // PID is tuned
//            std::cout << std::fixed;
//            std::cout << "Iter: " << pid_steering.iter_ << "\tCTE: " << cte << "\tSteering: " << steer_value << "\tThrottle: " << throttle_value << std::endl;
	  }


          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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
