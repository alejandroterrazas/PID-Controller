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

unsigned seed = 32;
static std::default_random_engine generator (seed);
static std::default_random_engine generator2 (seed);

//std::uniform_int_distribution<> dis(1, 6);

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


void print_vals(string msg) {
  std::cout << msg << "; parameter index: " << parameter_index << std::endl;
  std::cout << "p: " << p[0] << "," << p[1] << "," << p[2] << std::endl;
  std::cout << "dp: " << dp[0] << "," << dp[1] << "," << dp[2] << std::endl;  
}


bool first_part(int index, double error) {

  bool flag;
  if (error < best_error) {  
    best_error = error;
    dp[index] *= up_multiplier;
    print_vals("first part: error is better");
    flag = true;
  } else { //!error is worse       
    p[index] -= 2*dp[index];
    dp[index] *= down_multiplier;
    print_vals("first part error is worse");
    flag = false;
  }
  return flag;
}

bool second_part(int index, double error) {
  if (error < best_error) {
    best_error = error;
    dp[index] *= up_multiplier;
    print_vals("second part: error is better");
  } else {
    p[index] += dp[parameter_index];
    dp[index] *= down_multiplier;
    print_vals("error > best_error--set first = true");
  }  //end of best_error
  return true; //resets flag  
}

int generate_frame_skip(int frame_max) {
  std::uniform_int_distribution<>int_distribution(1,frame_max);
  return int_distribution(generator);
}

int generate_offset_skip(int offset_max) {
  std::uniform_int_distribution<>offset_distribution(1,offset_max);
  return offset_distribution(generator2);
 }

int main(int argc, char *argv[]) {
  uWS::Hub h;

  PID pid;
  
  steering_Kp_init = atof(argv[1]);
  steering_Ki_init = atof(argv[2]);
  steering_Kd_init = atof(argv[3]); 
  
  p = {steering_Kp_init, steering_Ki_init, steering_Kd_init}; 
  delta_p = {steering_Kp_init*0.1, steering_Ki_init*0.1, steering_Kd_init*0.1};
 
  offset = atoi(argv[4]);
  interval = atoi(argv[5]);
  max_n = atoi(argv[6]); 
  
  std::cout  << "steering_Kp_init: " << steering_Kp_init 
             << " steering_Ki_init: " << steering_Ki_init 
             << " steering_Kd_init: " << steering_Kd_init
             << " offset: " << offset 
             << " interval_skip: " << interval 
             << " max_n: " << max_n << std::endl;
  
  
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          // double speed = std::stod(j[1]["speed"].get<string>());
          // double angle = std::stod(j[1]["steering_angle"].get<string>());
          
          if (reset) {
            //check sum of dp
            double dpsum = accumulate(delta_p.begin(),delta_p.end(),0.0);  //use 0.0 or it will use int
           
            // std::cout << "dp sum: " << dpsum << std::endl;                       
            if (dpsum < .01) {
              std::cout << "DONE!" << std::endl;
              exit(0);
            }
           
            //up_multiplier and down_multiplier adjust the rate of weight changes
            up_multiplier -= (up_multiplier - 1)/10;
            down_multiplier += (1-down_multiplier)/10;
            pid.Init(p[0],p[1],p[2]);
            
            //pid.UpdateError(cte);
             
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), 
                    uWS::OpCode::TEXT); 
             
            
            int_skip = generate_frame_skip(interval);  //geneate random number of images between samples 
            offset_skip = generate_offset_skip(offset); //geneate random number of images to offset start
          
            std::cout << "sum of dp: " << dpsum 
                      << " interval skip: " << int_skip 
                      << " offset_skip: " << offset_skip 
                      << " int_skip: " << int_skip  
                      << " up_multiplier: " << up_multiplier 
                      << " down_multiplier: " << down_multiplier << std::endl;
            
            reset = false;
            offset_count = 0;
            count = 0;
          } 
          
          pid.UpdateError(cte);
          double steer_value = pid.TotalError();
          
          steer_value = (steer_value > 1) ? 1 : steer_value;
          steer_value  = (steer_value < -1) ? -1 : steer_value;
          //start counting until offset (starting point for sampling)
          offset_count += 1;
          //std::cout << "offset_count: " << offset_count << "offset_skip: " << offset_skip << std::endl;
          if (offset_count > offset_skip) {count += 1;}
            
          //std::cout << "count: " << count << " max_n: " << static_cast<unsigned int>(max_n) << std::endl;
          //count += 1;
          //once we reach the interval skip, take a sample
          if (count == int_skip) {
             error_vals.push_back(pow(cte,2)/static_cast<double>(max_n));
             count = 0;
          }
            
          if (error_vals.size() == static_cast<unsigned int>(max_n)) {
            double err = accumulate(error_vals.begin(), error_vals.end(), 0);
            //see if first part of twiddle is better; if not pass it to the seocond part
            if (first==true) { 
                first = first_part(parameter_index, err);
            } else {
                //first part didn't improve error, try the second part;
                first = second_part(parameter_index, err);  //function always returns true 
            }
            //we're done now.  Rset everything and go to the next parameter
            reset = true;
            error_vals.clear();
            parameter_index += 1;
            parameter_index = (parameter_index>2) ? 0 : parameter_index;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
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