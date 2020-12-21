#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
  public:
  // Constructors
  Vehicle();
  Vehicle(int lane, float s, float v, float a, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Public methods
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, 
                                      map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> lane_change_trajectory(string state, 
                                         map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, 
                                              map<int, vector<Vehicle>> &predictions);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane, 
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane, 
                         Vehicle &rVehicle);

  void realize_next_state(vector<Vehicle> &trajectory);
  
  vector<Vehicle> generate_predictions(int horizon=2);

  void configure(vector<double> &road_data);

  void update_localisation_data(vector<double> &local_data);

  void update_previous_path_data(vector<double> &previous_path_x, 
                                 vector<double> &previous_path_y, 
                                 double end_path_s, 
                                 double end_path_d);

  // Public variables
  string state;

  double lane, a, target_speed, goal_s, goal_lane;

  double x, y, s, d, yaw, speed; // localisation data

  vector<double> previous_path_x, previous_path_y; // previous path data
  double end_path_s, end_path_d; // previous path data

};

#endif // VEHICLE_H
