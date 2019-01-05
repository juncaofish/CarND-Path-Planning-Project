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

struct Vehicle {
    double d;
    double vx, vy;
    double speed;
    double s;

    Vehicle(nlohmann::json sensor_fusion) {
        this->vx = sensor_fusion[3];
        this->vy = sensor_fusion[4];
        this->s = sensor_fusion[5];
        this->d = sensor_fusion[6];
        this->speed = sqrt(vx * vx + vy * vy);
    }
};


// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi / 180; }

double rad2deg(double x) { return x * 180 / pi; }

// Calculate the Euclidea Distance between two points
double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Check if a vehicle is is a certain lane
bool is_in_lane(double d, int lane) {
    return (d > lane_width * lane) && (d < lane_width * lane + lane_width);
}

// Choose in the map of highway waypoints the one closest to the car
int get_closest_waypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

    double closest_len;
    closest_len = 1000000;
    int closest_waypoint = 0;

    for (int i = 0; i < maps_x.size(); i++) {

        double map_x = maps_x[i];
        double map_y = maps_y[i];

        double dist = distance(x, y, map_x, map_y);
        if (dist < closest_len) {
            closest_len = dist;
            closest_waypoint = i;
        }
    }
    return closest_waypoint;
}

// Choose in the map of highway waypoints the closest before the car (that is the next).
// The actual closest waypoint could be behind the car.
int
get_next_waypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

    int closest_waypoint = get_closest_waypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closest_waypoint];
    double map_y = maps_y[closest_waypoint];

    double heading = atan2(map_y - y, map_x - x);

    double angle = abs(theta - heading);

    if (angle > pi / 4)
        closest_waypoint++;

    return closest_waypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double>
get_frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
    int next_wp = get_next_waypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
        prev_wp = maps_x.size() - 1;

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // Find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    // See if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
        frenet_d *= -1;

    // Calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++)
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double>
get_cartesian(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x,
              const vector<double> &maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1)))
        prev_wp++;

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

#endif