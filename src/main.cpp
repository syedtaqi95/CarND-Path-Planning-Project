#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "trajectory.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::map;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  // Some useful variables
  double goal_s = max_s;
  double goal_lane = 1; // lane = 0 (left), 1 (middle), 2 (right)  
  double lane = 1; // current lane 
  double ref_vel = 0.; // reference velocity to target (mph)
  double SPEED_LIMIT = 49.5; // max velocity (mph)
  double MIN_GAP = 20.; // minimum gap between ego and front obstacle vehicle in lane

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, 
               &SPEED_LIMIT, &MIN_GAP]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          // [ id, x, y, vx, vy, s, d]
          auto sensor_fusion = j[1]["sensor_fusion"];

          //Code from classroom walkthrough

          // Behavioural planner
          // Generate ref vel and target lane using sensor fusion and localisation data

          int prev_size = previous_path_x.size();
          
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;

          // Calculate ref velocity by checking other vehicles (obstacles)
          for(int i = 0; i < sensor_fusion.size(); i++) {
            
            double obs_d = sensor_fusion[i][6];

            // if obstacle is within ego lane boundaries
            if ( (obs_d < (2+4*lane+2)) && (obs_d > (2+4*lane-2)) ) {
              
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double obs_speed = sqrt(pow(vx,2) + pow(vy,2));
              double obs_s_pred = sensor_fusion[i][5];

              // Predict obs s up to previous path size
              obs_s_pred += (double)prev_size * 0.02 * obs_speed;

              // if obs_s is greater than ego s and if gap is too close
              if ( (obs_s_pred > car_s) && ((obs_s_pred - car_s) < MIN_GAP) ) {
                
                // Do some logic here
                too_close = true;

              }
            }
          }

          if(too_close) {
            // reduce ref vel by max allowed accel (5 m/s2)
            ref_vel -= 0.224;
          }
          else if(ref_vel < SPEED_LIMIT) {
            // increase ref_vel by max allowed accel (5 m/s2)
            ref_vel += 0.224;
          }

          // Trajectory generation          
          vector<vector<double>> next_vals = trajectory_gen( 
                                            previous_path_x, 
                                            previous_path_y, 
                                            car_x, car_y, car_yaw,
                                            car_s, lane, 
                                            map_waypoints_x,
                                            map_waypoints_y,
                                            map_waypoints_s,
                                            ref_vel );

          /******************************************************************/

          json msgJson;

          msgJson["next_x"] = next_vals[0];
          msgJson["next_y"] = next_vals[1];

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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