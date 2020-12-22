#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "helpers.h"
#include "spline.h"

// for convenience
using std::vector;



vector<vector<double>> generate_trajectory( const vector<double> &previous_path_x, 
                                       const vector<double> &previous_path_y, 
                                       double &car_x, double &car_y, double &car_yaw,
                                       double &car_s, double &target_lane, 
                                       vector<double> &map_waypoints_x,
                                       vector<double> &map_waypoints_y,
                                       vector<double> &map_waypoints_s,
                                       double &ref_vel );

#endif // PLANNER_H