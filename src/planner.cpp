#include "planner.h"

using std::vector;
using std::map;

void behaviour_planner( const vector<double> &previous_path_x, 
                        const vector<double> &previous_path_y, 
                        double &current_lane,
                        double &car_d, double &car_s,
                        double &end_path_s,
                        const vector<vector<double>> &sensor_fusion,
                        double &ref_vel,
                        string &current_state,
                        string &next_state) {

  // Useful variables
  double SPEED_LIMIT = 49.5; // max velocity (mph)
  double MIN_GAP = 20.; // minimum gap between ego and front obstacle vehicle in lane
  int prev_size = previous_path_x.size();
  current_lane = floor(car_d / 4);
  
  if (prev_size > 0) {
    car_s = end_path_s;
  }

  bool too_close = false;

  // Create vectors to store the non-ego vehicle data in each lane
  vector<double> vehicles_in_lane0;
  vector<double> vehicles_in_lane1;
  vector<double> vehicles_in_lane2;

  double speed_car_ahead = 0.;

  // Loop through sensor fusion
  for(int i = 0; i < sensor_fusion.size(); i++) {
    
    double obs_d = sensor_fusion[i][6];
    double obs_lane = floor(obs_d/4);
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double obs_speed = sqrt(pow(vx,2) + pow(vy,2));
    double obs_s = sensor_fusion[i][5];

    obs_s += (double)prev_size * 0.02 * obs_speed;

    if (obs_lane == 0) { // Lane 0
      if (abs(obs_s - car_s) <= 20) { // If within 20m of ego
        vehicles_in_lane0.push_back(i);
      }      
    }
    else if (obs_lane == 1) { // Lane 1
      if (abs(obs_s - car_s) <= 20) { // If within 20m of ego
        vehicles_in_lane1.push_back(i);
      }      
    }
    else if (obs_lane == 2) { // Lane 2
      if (abs(obs_s - car_s) <= 20) { // If within 20m of ego
        vehicles_in_lane2.push_back(i);
      }      
    }

    // if in same lane as ego
    if ( obs_lane == current_lane ) {
      speed_car_ahead = obs_speed;
      // if obs_s is greater than ego s and if gap is less than threshold, flag too close
      if ( (obs_s > car_s) && ((obs_s - car_s) < MIN_GAP) ) {        
        too_close = true;        
      }
    }
  } 

  if(too_close) {
    // Lane change cases

    // reduce ref vel by max allowed accel (5 m/s2)
    if(ref_vel >= speed_car_ahead){
        ref_vel -= 0.224;
        std::cout << "car_s: " << car_s << " Front vehicle is close, ego slowing down" << std::endl;
    }
    //Try to maintain the speed of car in front
    else if(ref_vel < speed_car_ahead){
        ref_vel += 0.224;
        std::cout << "car_s: " << car_s << " Front vehicle is too close, ego speeding up" << std::endl;
    }
  }
  else if(ref_vel < SPEED_LIMIT) {
    // increase ref_vel by max accel
    ref_vel += 0.224;
  }

} // end behaviour_planner


//Use the ref_vel, target lane info to generate a smooth trajectory using spline
vector<vector<double>> generate_trajectory( const vector<double> &previous_path_x, 
                                            const vector<double> &previous_path_y, 
                                            double &car_x, double &car_y, double &car_yaw,
                                            double &car_s, double &target_lane, 
                                            vector<double> &map_waypoints_x,
                                            vector<double> &map_waypoints_y,
                                            vector<double> &map_waypoints_s,
                                            double &ref_vel ) {
  
  // Useful variables                                       
  int prev_size = previous_path_x.size();
  double ANCHOR_WPS_DIST = 30.; // generate sparse waypoints spaced this far apart
  double dt = 0.02; // time in seconds between path points

  // x,y points used in the output
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Load previous path points into next_x/y_vals
  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Create a vector of sparsely spaced waypoints
  // Use this to create a spline to generate the trajectory
  vector<double> anchor_points_x;
  vector<double> anchor_points_y;

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
    anchor_points_x.push_back(prev_car_x);
    anchor_points_x.push_back(car_x);
    anchor_points_y.push_back(prev_car_y);
    anchor_points_y.push_back(car_y);
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
    anchor_points_x.push_back(ref_x_prev);
    anchor_points_x.push_back(ref_x);
    anchor_points_y.push_back(ref_y_prev);
    anchor_points_y.push_back(ref_y);

  }

  // Add wp's spaced ANCHOR_WPS_DIST apart in s, in the same target_lane (d) 
  // Using highly spaced wp's to minimise jerk and acceleration
  for (int i = 1; i < 4; i++) {
    vector<double> next_wp = getXY(car_s + i*ANCHOR_WPS_DIST, 2 + target_lane*4, map_waypoints_s, 
                                map_waypoints_x, map_waypoints_y);
    anchor_points_x.push_back(next_wp[0]);
    anchor_points_y.push_back(next_wp[1]);
  }

  // Convert from global to local ego coordinates
  for (int i = 0; i < anchor_points_x.size(); i++) {
  
    // shift points so ego vehicle is at origin
    double shift_x = anchor_points_x[i] - ref_x;
    double shift_y = anchor_points_y[i] - ref_y;

    // rotate so that reference angle is 0 degrees
    anchor_points_x[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
    anchor_points_y[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
  }

  // Create a spline
  tk::spline sp;

  // Set x,y points in spline
  sp.set_points(anchor_points_x, anchor_points_y);          

  // Calculate how to break up our spline so we travel at the ref velocity
  double target_x = 30.;
  double target_y = sp(target_x);
  double target_dist = sqrt( pow(target_x,2) + pow(target_y,2) );

  double x_add_on = 0;
  double N = target_dist / (dt * ref_vel / 2.24); //2.24 converts mph to m/s

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

  vector<vector<double>> output_vector;
  output_vector.push_back(next_x_vals);
  output_vector.push_back(next_y_vals);

  return output_vector;

} // end generate_trajectory