#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <map>
#include <iostream>
#include "helpers.h"
#include "spline.h"

// for convenience
using std::vector;
using std::map;

// map<double, vector<double>> generate_predictions(const vector<vector<double>> &sensor_fusion,
//                                                  int horizon=50);


void behaviour_planner( const vector<double> &previous_path_x, 
                        const vector<double> &previous_path_y, 
                        double &current_lane,
                        double &target_lane,
                        double &car_d, double &car_s,
                        double &end_path_s,
                        const vector<vector<double>> &sensor_fusion,
                        double &ref_vel,
                        string &current_state,
                        string &next_state);

vector<vector<double>> generate_trajectory( const vector<double> &previous_path_x, 
                                            const vector<double> &previous_path_y, 
                                            double &car_x, double &car_y, double &car_yaw,
                                            double &car_s, double &target_lane, 
                                            vector<double> &map_waypoints_x,
                                            vector<double> &map_waypoints_y,
                                            vector<double> &map_waypoints_s,
                                            double &ref_vel );

#endif // PLANNER_H