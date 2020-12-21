#ifndef COST_H
#define COST_H

using std::map;
using std::string;
using std::vector;

float calculate_cost();

float goal_distance_cost();

float inefficiency_cost();

float lane_speed();

map<string, float> get_helper_data();

#endif  // COST_H