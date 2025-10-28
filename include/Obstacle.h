#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <string>
#include <vector>

struct Obstacle {
    double x, y, yaw, front, rear, width, vx, vy, vyaw;
};

double mod2pi(double x)
{
    double v = std::fmod(x, 2.0 * M_PI);
    if (x < 0)
        v = std::fmod(x, -2.0 * M_PI);
    if (v < -M_PI)
        v += 2.0 * M_PI;
    else if (v > M_PI)
        v -= 2.0 * M_PI;
    return v;
}

struct Waypoint {
    double time;
    double vx;
    double steer;
    double x, y, yaw;
    bool is_present = true;

    Waypoint(double time_ = 0.0, double vx_ = 0.0, double steer_ = 0.0, double x_ = 0.0, double y_ = 0.0, double yaw_ = 0.0, bool present_ = true)
        : time(time_), vx(vx_), steer(steer_), x(x_), y(y_), yaw(yaw_), is_present(present_) {}
};

struct DynamicObstacle {
    std::string name;
    double start_time;
    double front, rear, width;
    std::vector<Waypoint> path;
};

std::tuple<double, double, double, bool> get_ob_pose(const DynamicObstacle& ob, double global_t) {
    double r_t = global_t - ob.start_time;
    if (ob.path.empty()) {
        return {0.0, 0.0, 0.0, false};
    }
    if (r_t < ob.path[0].time) {
        const auto& wp = ob.path[0];
        return {wp.x, wp.y, wp.yaw, wp.is_present};
    }
    if (r_t >= ob.path.back().time) {
        const auto& wp = ob.path.back();
        return {wp.x, wp.y, wp.yaw, wp.is_present};
    }
    auto it = std::lower_bound(ob.path.begin(), ob.path.end(), r_t, [](const Waypoint& wp, double t) { return wp.time < t; });
    size_t j = std::distance(ob.path.begin(), it);
    if (j < ob.path.size() && ob.path[j].time == r_t) {
        const auto& wp = ob.path[j];
        return {wp.x, wp.y, wp.yaw, wp.is_present};
    }
    size_t i = j - 1;
    double frac = (r_t - ob.path[i].time) / (ob.path[i + 1].time - ob.path[i].time);
    double x = ob.path[i].x + frac * (ob.path[i + 1].x - ob.path[i].x);
    double y = ob.path[i].y + frac * (ob.path[i + 1].y - ob.path[i].y);
    double dyaw = mod2pi(ob.path[i + 1].yaw - ob.path[i].yaw);
    double yaw = ob.path[i].yaw + frac * dyaw;
    yaw = mod2pi(yaw);
    bool is_present = ob.path[i].is_present && ob.path[i + 1].is_present;
    return {x, y, yaw, is_present};
}

#endif // OBSTACLE_H
