//
// Created by Mark on 2/9/18.
//

#ifndef PATH_PLANNING_MAP_HELPER_H
#define PATH_PLANNING_MAP_HELPER_H

#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;

class Map {

private:
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

public:
    Map();

    virtual ~Map();

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    void load_map(string map_file);

    // For converting back and forth between radians and degrees.
    constexpr double pi() { return M_PI; }
    double deg2rad(double x) { return x * pi() / 180; }
    double rad2deg(double x) { return x * 180 / pi(); }

    double distance(double x1, double y1, double x2, double y2);

    int ClosestWaypoint(double x, double y);

    int NextWaypoint(double x, double y, double theta);

    vector<double> getFrenet(double x, double y, double theta);

    vector<double> getXY(double s, double d);
};

#endif //PATH_PLANNING_MAP_HELPER_H
