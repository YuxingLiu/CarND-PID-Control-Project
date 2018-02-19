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
  // Initialize the pid variable (Kp, Ki, Kd, lb, ub).
  pid_steering.Init(0.12, 0.0, 3.0, -1.0, 1.0);
  pid_throttle.Init(0.1, 0.0001, 2.0, 0.0, 1.0);

  // Flag, if steering PID is tuned
  bool is_pid_tuned = 0;
  std::vector<double> Kp_array = {0.08, 0.1, 0.12};
  std::vector<double> Zd_array = {20, 25, 30, 35, 40};
  int epoch = 0;

  h.onMessage([&pid_steering, &pid_throttle, &is_pid_tuned, &Kp_array, &Zd_array, &epoch](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

	  // Steering control
	  pid_steering.UpdateError(-cte);
	  double steer_value = pid_steering.TotalError();

          // Throttle control;
          double speed_ref = 40.0;
          double error_speed = speed_ref - speed;
          pid_throttle.UpdateError(error_speed);
          double throttle_value = pid_throttle.TotalError();

	  // Steering PID tuning
	  if(!is_pid_tuned && pid_steering.iter_ == 260) {
            std::cout << std::fixed;
            std::cout << "Epoch: " << epoch << "\tKp: " << pid_steering.Kp_ << "\tKd: " << pid_steering.Kd_ << "\tIter: " << pid_steering.iter_ << "\tMSE: " << pid_steering.mse_ << "\tMax cte: " << pid_steering.max_cte_ << std::endl;
	    std::cout << std::endl;

	    if(epoch < Kp_array.size()*Zd_array.size()) {
	      int i_p = epoch / Zd_array.size();
	      int i_d = epoch % Zd_array.size();
	      double Kp = Kp_array[i_p];
	      double Kd = Kp_array[i_p] * Zd_array[i_d];

	      pid_steering.Init(Kp, 0.0, Kd, -1.0, 1.0);
	      epoch += 1;
	    }else {
	      is_pid_tuned = 1;
	    }
	  }else if(is_pid_tuned) {  // PID is tuned
//            std::cout << std::fixed;
//	    std::cout << "Iter: \t" << pid_steering.iter_ << "\t\t MSE: \t\t" << pid_steering.mse_ << "\t Max cte: \t" << pid_steering.max_cte_ << std::endl;
//            std::cout << "CTE: \t" << cte << "\t Steering: \t" << steer_value << "\t Throttle: \t" << throttle_value << std::endl;
//	    std::cout << std::endl;
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
