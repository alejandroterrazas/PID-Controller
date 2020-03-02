#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <random>
#include "json.hpp"
#include "PID.h"
#include "helpers.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[]) {
  uWS::Hub h;

  PID steering_pid;
  PID throttle_pid;
  
  steering_Kp_init = atof(argv[1]);
  steering_Ki_init = atof(argv[2]);
  steering_Kd_init = atof(argv[3]); 
  
  throttle_Kp_init = atof(argv[4]);
  throttle_Ki_init = atof(argv[5]);
  throttle_Kd_init = atof(argv[6]); 
  
  std::cout  << "steering_Kp_init: " << steering_Kp_init 
             << " steering_Ki_init: " << steering_Ki_init 
             << " steering_Kd_init: " << steering_Kd_init
             << " throttle_Kp_init: " << throttle_Kp_init 
             << " throttle_Ki_init: " << throttle_Ki_init 
             << " throttle_Kd_init: " << throttle_Kd_init << std::endl;
   
  h.onMessage([&steering_pid, &throttle_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry" && done == false) {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          //double speed = std::stod(j[1]["speed"].get<string>());
          // double angle = std::stod(j[1]["steering_angle"].get<string>());

          
          if (initialize) {
            
            steering_pid.Init(steering_Kp_init, steering_Ki_init , steering_Kd_init);
            throttle_pid.Init(throttle_Kp_init, throttle_Ki_init , throttle_Kd_init);
            
            initialize = false;
          }
          
          steering_pid.UpdateError(cte);
          double steer_value = steering_pid.TotalError();
          
          steer_value = (steer_value > 1) ? 1 : steer_value;
          steer_value  = (steer_value < -1) ? -1 : steer_value;
         // std::cout << "steer value: " << steer_value << std::endl;
          
          throttle_pid.UpdateError(cte);  //n.b. use abs with throttle
          double throttle_value = .4 + throttle_pid.TotalError();
          
          throttle_value = (throttle_value < .1) ? .1 : throttle_value;  //throttle min of .1
         // throttle_value = .3;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}