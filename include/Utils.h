#ifndef UTILS_H
#define UTILS_H

#include <cmath>

inline double mod2pi(double x) {
    double v = std::fmod(x, 2.0 * M_PI);
    if (x < 0) v = std::fmod(x, -2.0 * M_PI);
    if (v < -M_PI) v += 2.0 * M_PI;
    else if (v > M_PI) v -= 2.0 * M_PI;
    return v;
}

inline double pi_2_pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

#endif // UTILS_H
