//
// Created by Mark on 2/9/18.
//

#ifndef PATH_PLANNING_MAP_HELPER_H
#define PATH_PLANNING_MAP_HELPER_H

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;


class Map {

private:

public:
    Map();

    virtual ~Map();

    double distance(double x1, double y1, double x2, double y2);

    int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

    int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

    vector<double> getFrenet(double x, double y, double theta,
                             const vector<double> &maps_x, const vector<double> &maps_y);
    vector<double> getXY(double s, double d,
                         const vector<double> &maps_s,
                         const vector<double> &maps_x, const vector<double> &maps_y);
};

#endif //PATH_PLANNING_MAP_HELPER_H
