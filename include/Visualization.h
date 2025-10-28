/*************************************
 * Path Visualization using Qt
 *
 * 2025.10.24
 * Jeeho Ahn, jeeho@umich.edu
*************************************/


#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QPainter>
#include <QSlider>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QLabel>

#include <vector>

#include <Point.h>
#include <Obstacle.h>
#include <Params.h>




std::tuple<double, double, double> interpolate_timed_path(const std::vector<std::tuple<double, double, double, double>>& timed_path, double t) {
    if (t <= std::get<0>(timed_path[0])) return {std::get<1>(timed_path[0]), std::get<2>(timed_path[0]), std::get<3>(timed_path[0])};
    if (t >= std::get<0>(timed_path.back())) return {std::get<1>(timed_path.back()), std::get<2>(timed_path.back()), std::get<3>(timed_path.back())};
    for (size_t i = 0; i < timed_path.size() - 1; ++i) {
        auto [t1, x1, y1, yaw1] = timed_path[i];
        auto [t2, x2, y2, yaw2] = timed_path[i + 1];
        if (t1 <= t && t <= t2) {
            double frac = (t2 > t1) ? (t - t1) / (t2 - t1) : 0;
            double x = x1 + frac * (x2 - x1);
            double y = y1 + frac * (y2 - y1);
            double dyaw = mod2pi(yaw2 - yaw1);
            double yaw = yaw1 + frac * dyaw;
            yaw = mod2pi(yaw);
            return {x, y, yaw};
        }
    }
    return {std::get<1>(timed_path.back()), std::get<2>(timed_path.back()), std::get<3>(timed_path.back())};
}


class VizWidget : public QWidget {
private:
    const std::vector<std::tuple<double, double, double, double>>& timed_path;
    const std::vector<DynamicObstacle>& obstacles;
    const Params& params;
    double current_t = 0.0;
    double max_t = 0.0;

    void drawFilledPolygon(QPainter& p, const Corners& corners, const QColor& color, const std::function<double(double)>& screen_x, const std::function<double(double)>& screen_y) {
        QPolygonF poly;
        for (const auto& c : corners) poly << QPointF(screen_x(c.x), screen_y(c.y));
        poly << QPointF(screen_x(corners[0].x), screen_y(corners[0].y));
        p.setPen(color);
        p.setBrush(color);
        p.drawPolygon(poly);
    }

public:
    VizWidget(const std::vector<std::tuple<double, double, double, double>>& path, const std::vector<DynamicObstacle>& obs, const Params& p)
        : timed_path(path), obstacles(obs), params(p) {
        if (!timed_path.empty()) max_t = std::get<0>(timed_path.back());
        setMinimumSize(600, 600);
    }

    void setTime(double t) {
        current_t = t;
        update();
    }

protected:
    void paintEvent(QPaintEvent* event) override {
        QPainter p(this);
        p.fillRect(rect(), Qt::white);

        double scale_x = static_cast<double>(width()) / (params.max_x - params.min_x);
        double scale_y = static_cast<double>(height()) / (params.max_y - params.min_y);
        double sc = std::min(scale_x, scale_y);
        auto screen_x = [&](double x) { return (x - params.min_x) * sc; };
        auto screen_y = [&](double y) { return (params.max_y - y) * sc; };

        // Draw boundary a little inside
        p.setPen(Qt::black);
        p.drawRect(QRectF(screen_x(params.min_x) + 0.5, screen_y(params.max_y) + 0.5, sc * (params.max_x - params.min_x) - 1, sc * (params.max_y - params.min_y) - 1));

        // Draw grid
        p.setPen(Qt::lightGray);
        for (double x = params.min_x; x <= params.max_x + 1e-6; x += params.xy_resolution) {
            double sx = screen_x(x);
            double sy1 = screen_y(params.min_y);
            double sy2 = screen_y(params.max_y);
            p.drawLine(QPointF(sx, sy1), QPointF(sx, sy2));
        }
        for (double y = params.min_y; y <= params.max_y + 1e-6; y += params.xy_resolution) {
            double sy = screen_y(y);
            double sx1 = screen_x(params.min_x);
            double sx2 = screen_x(params.max_x);
            p.drawLine(QPointF(sx1, sy), QPointF(sx2, sy));
        }

        // Draw path
        p.setPen(QPen(Qt::red, 2));
        QPolygonF path_poly;
        for (const auto& pt : timed_path) {
            path_poly << QPointF(screen_x(std::get<1>(pt)), screen_y(std::get<2>(pt)));
        }
        p.drawPolyline(path_poly);

        // Draw robot
        auto [rx, ry, ryaw] = interpolate_timed_path(timed_path, current_t);
        auto robot_corners = get_corners(rx, ry, ryaw, params.robot_front_length, params.robot_rear_length, params.robot_width);
        drawFilledPolygon(p, robot_corners, QColor("#555B6E"), screen_x, screen_y);

        // Indicate robot front with a red arrow
        double front_x = rx + params.robot_front_length * std::cos(ryaw);
        double front_y = ry + params.robot_front_length * std::sin(ryaw);
        p.setPen(QPen(Qt::red, 2));
        p.drawLine(screen_x(rx), screen_y(ry), screen_x(front_x), screen_y(front_y));

        // Draw obstacles
        for (const auto& ob : obstacles) {
            auto [ox, oy, oyaw, is_present] = get_ob_pose(ob, current_t);
            if (!is_present) continue;
            auto ob_corners = get_corners(ox, oy, oyaw, ob.front, ob.rear, ob.width);
            drawFilledPolygon(p, ob_corners, QColor("#89B0AE"), screen_x, screen_y);
        }
    }
};


#endif // VISUALIZATION_H
