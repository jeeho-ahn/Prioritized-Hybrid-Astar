#ifndef POINT_H
#define POINT_H

#include <vector>

struct Point {
    double x, y;
};

using Corners = std::vector<Point>;

Corners get_corners(double x, double y, double yaw, double front, double rear, double width) {
    double cos = std::cos(yaw);
    double sin = std::sin(yaw);
    Corners corners = {
        {x - rear * cos - (width / 2) * sin, y - rear * sin + (width / 2) * cos},
        {x + front * cos - (width / 2) * sin, y + front * sin + (width / 2) * cos},
        {x + front * cos + (width / 2) * sin, y + front * sin - (width / 2) * cos},
        {x - rear * cos + (width / 2) * sin, y - rear * sin - (width / 2) * cos}
    };
    return corners;
}


#endif // POINT_H
