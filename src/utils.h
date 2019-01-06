#ifndef UTILS_H
#define UTILS_H

#include "stdio.h"
#include <math.h>
#include <limits>
#include <vector>
#include "json.hpp"

const double pi = M_PI;

const double lane_width = 4.0;        // width of a lane					(m)
const double safety_margin = 25.0;        // distance to keep from other cars	(m)
const double max_safe_speed = 49.0;        // max reference speed in the limit	(mph)

using namespace std;

// Modelling of a vehicle
struct Vehicle {
    double d;   // Frenet coordinate: lateral displacement
    double vx, vy;
    double speed;
    double s;   // Frenet coordinate: longitudinal displacement

    Vehicle(nlohmann::json sensor_fusion) {
        this->vx = sensor_fusion[3];
        this->vy = sensor_fusion[4];
        this->s = sensor_fusion[5];
        this->d = sensor_fusion[6];
        this->speed = sqrt(vx * vx + vy * vy);
    }
};

// Check if a vehicle is is a certain lane
bool is_in_lane(double d, int lane) {
    return (d > lane_width * lane) && (d < lane_width * (lane + 1));
}

#endif