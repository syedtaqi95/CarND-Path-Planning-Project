#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  double ref_vel = 0.224; // reference velocity to target (mph), init to 10 m/s i.e. max accel
  double SPEED_LIMIT = 49.5; // max velocity (mph)
  double MIN_GAP = 20.0; // minimum gap size between ego and front obstacle vehicle in lane
  double SPARSE_WP_DIST = 30.0; // generate sparse waypoints spaced this far apart

  // Create ego vehicle
  Vehicle ego = Vehicle(lane, 0, 0, 0);

  // Configure ego vehicle  
  // configuration data: speed limit, num_lanes, goal_s, goal_lane
  double num_lanes = 3;
  vector<double> ego_config = {SPEED_LIMIT,num_lanes,goal_s,goal_lane};
  ego.configure(ego_config);
  ego.state = "KL";

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, 
               &SPEED_LIMIT, &MIN_GAP, &SPARSE_WP_DIST]
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
          auto sensor_fusion = j[1]["sensor_fusion"];

          /******************************************************************/

          int prev_size = previous_path_x.size();

          // Behavioural planner
          // Generate ref vel and target lane using sensor fusion and localisation data
          
          double car_s_pred = car_s;
          if (prev_size > 0) {
            car_s_pred = end_path_s;
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
              if ( (obs_s_pred > car_s_pred) && ((obs_s_pred - car_s_pred) < MIN_GAP) ) {
                
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
          // Use the ref_vel, target lane info to generate a smooth trajectory using spline

          // Actual x,y points used in the output
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Load previous path points into next_x/y_vals
          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Create a vector of sparsely spaced waypoints
          // Use this to create a spline to generate the trajectory
          vector<double> sparse_wp_x;
          vector<double> sparse_wp_y;

          // Reference x, y, yaw of ego vehicle
          // Either set to ego's current pos or at the previous path's end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw); //rad

          // if previous points are almost empty, use the ego as starting ref
          if (prev_size < 2) {
            
            // Use a point tangent to the ego
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            // Add to sparse wp vectors
            sparse_wp_x.push_back(prev_car_x);
            sparse_wp_x.push_back(car_x);
            sparse_wp_y.push_back(prev_car_y);
            sparse_wp_y.push_back(car_y);
          }
          else {
              
            // Set ref points to previous path's end point and calculate the yaw
            // This is a more accurate reading than simply using the localisation data which
            // is out of date 
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Add to sparse wp vectors
            sparse_wp_x.push_back(ref_x_prev);
            sparse_wp_x.push_back(ref_x);
            sparse_wp_y.push_back(ref_y_prev);
            sparse_wp_y.push_back(ref_y);

          }

          // Add wp's spaced SPARSE_WP_DIST apart in s, in the same lane (d) 
          // Using highly spaced wp's to minimise jerk and acceleration
          for (int i = 1; i < 4; i++) {
            vector<double> next_wp = getXY(car_s + i*SPARSE_WP_DIST, 2 + lane*4, map_waypoints_s, 
                                      map_waypoints_x, map_waypoints_y);
            sparse_wp_x.push_back(next_wp[0]);
            sparse_wp_y.push_back(next_wp[1]);
          }

          // Convert from global to local ego coordinates
          for (int i = 0; i < sparse_wp_x.size(); i++) {
            
            // shift points so ego vehicle is at origin
            double shift_x = sparse_wp_x[i] - ref_x;
            double shift_y = sparse_wp_y[i] - ref_y;

            // rotate so that reference angle is 0 degrees
            sparse_wp_x[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            sparse_wp_y[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          // Create a spline
          tk::spline sp;

          // Set x,y points in spline
          sp.set_points(sparse_wp_x, sparse_wp_y);          

          // Calculate how to break up our spline so we travel at the ref velocity
          double target_x = 30.0;
          double target_y = sp(target_x);
          double target_dist = sqrt( pow(target_x,2) + pow(target_y,2) );

          double x_add_on = 0;
          double N = target_dist / (0.02 * ref_vel / 2.24); //2.24 converts mph to m/s

          // Fill up next_x/y_vals so there are total 50 points
          for (int i = 1; i <= 50 - prev_size; i++) {
            
            double x_point = x_add_on + target_x/N;
            double y_point = sp(x_point);

            x_add_on = x_point;

            // Convert back from local to global coordinates
            double x_ref = x_point;
            double y_ref = y_point;
            x_point = x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          /******************************************************************/

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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