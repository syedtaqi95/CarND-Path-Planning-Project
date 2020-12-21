#include "vehicle.h"
using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {
  this->lane = lane;
  this->s = s;
  this->speed = v;
  this->a = a;
  this->state = state;
}

Vehicle::~Vehicle() {}

void Vehicle::configure(vector<double> &road_data) {
  // Sets various parameters which will impact the ego vehicle.
  this->target_speed = road_data[0];
  this->goal_s = road_data[1];
  this->goal_lane = road_data[2];
}

void Vehicle::update_localisation_data(vector<double> &local_data) {
  // Updates the ego localisation data on every update cycle
  this->x = local_data[0];
  this->y = local_data[1];
  this->s = local_data[2];
  this->d = local_data[3];
  this->yaw = local_data[4];
  this->speed = local_data[5];
}

void Vehicle::update_previous_path_data(vector<double> &previous_path_x, 
                                 vector<double> &previous_path_y, 
                                 double end_path_s, 
                                 double end_path_d) {
  // Updates the ego previous path variables
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->end_path_s = end_path_s;
  this->end_path_d = end_path_d;
}
