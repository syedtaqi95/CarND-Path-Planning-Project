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
  void configure(vector<double> &road_data);

  // Public variables
  string state;

  double lane, s, v, a, target_speed, lanes_available, goal_s, goal_lane;

};

#endif // VEHICLE_H
