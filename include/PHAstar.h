/*************************************
 * Prioritized Hybrid Astar
 *
 * 2025.10.24
 * Jeeho Ahn, jeeho@umich.edu
*************************************/

#ifndef PHASTAR_H
#define PHASTAR_H

#include <Point.h>
#include <Obstacle.h>

#include <Reeds_Shepp.h>
#include <Params.h>

// PHA* Implementation
std::pair<double, double> project(const Corners& corners, const Point& axis) {
    double min_d = std::numeric_limits<double>::infinity();
    double max_d = -std::numeric_limits<double>::infinity();
    for (const auto& c : corners) {
        double dot = c.x * axis.x + c.y * axis.y;
        min_d = std::min(min_d, dot);
        max_d = std::max(max_d, dot);
    }
    return {min_d, max_d};
}

bool overlap(std::pair<double, double> p1, std::pair<double, double> p2) {
    return p1.second >= p2.first && p2.second >= p1.first;
}

std::vector<Point> get_axes(const Corners& corners) {
    std::vector<Point> axes;
    for (size_t i = 0; i < 4; ++i) {
        Point p1 = corners[i];
        Point p2 = corners[(i + 1) % 4];
        double ex = p2.x - p1.x;
        double ey = p2.y - p1.y;
        Point normal = {-ey, ex};
        double norm = std::hypot(normal.x, normal.y);
        if (norm > 0) {
            normal.x /= norm;
            normal.y /= norm;
            axes.push_back(normal);
        }
    }
    return axes;
}

bool rectangles_intersect(const Corners& corners1, const Corners& corners2) {
    auto axes1 = get_axes(corners1);
    auto axes2 = get_axes(corners2);
    std::vector<Point> all_axes;
    all_axes.reserve(axes1.size() + axes2.size());
    all_axes.insert(all_axes.end(), axes1.begin(), axes1.end());
    all_axes.insert(all_axes.end(), axes2.begin(), axes2.end());
    for (const auto& axis : all_axes) {
        auto p1 = project(corners1, axis);
        auto p2 = project(corners2, axis);
        if (!overlap(p1, p2)) return false;
    }
    return true;
}

bool is_in_bounds(const Corners& corners, double min_x, double max_x, double min_y, double max_y) {
    for (const auto& c : corners) {
        if (!(min_x <= c.x && c.x <= max_x && min_y <= c.y && c.y <= max_y)) return false;
    }
    return true;
}


struct Node {
    double x, y, yaw, t, cost;
    Node* parent;
    int direction;
    Node(double x_ = 0, double y_ = 0, double yaw_ = 0, double t_ = 0, double cost_ = 0, Node* p = nullptr, int dir = 1)
        : x(x_), y(y_), yaw(yaw_), t(t_), cost(cost_), parent(p), direction(dir) {}
};

class PHAStar {
private:
    std::unique_ptr<Node> start, goal;
    std::vector<DynamicObstacle> obstacles;
    Params params;
    int x_width, y_width, theta_width, time_width;
    std::vector<double> ob_diags;
    std::vector<std::unique_ptr<Node>> all_nodes; // To manage memory

    size_t calc_grid_index(const Node* node) {
        int ix = static_cast<int>((node->x - params.min_x) / params.xy_resolution);
        int iy = static_cast<int>((node->y - params.min_y) / params.xy_resolution);
        int itheta = static_cast<int>(node->yaw / params.yaw_resolution) % theta_width;
        int itime = static_cast<int>(node->t / params.time_resolution);
        return (((static_cast<size_t>(iy) * x_width + ix) * theta_width + itheta) * time_width) + itime;
    }

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

    Node* generate_node(Node* current, std::pair<int, double> prim) {
        int direction = prim.first;
        double steer = prim.second;
        double x = current->x, y = current->y, yaw = current->yaw;
        double additional_cost = 0.0;
        double distance = 0.0;
        if (direction == 0) { // Wait
            additional_cost = params.wait_penalty;
        } else {
            additional_cost += (direction != current->direction) ? params.switch_penalty : 0.0;
            additional_cost += (direction < 0) ? params.reverse_penalty : 0.0;
            additional_cost += (std::abs(steer) > 1e-5) ? params.turn_penalty : 0.0;
            double d = direction * params.movement_length;
            distance = std::abs(d);
            double steer_adjusted = (direction > 0) ? steer : -steer;
            if (std::abs(steer_adjusted) < 1e-5) {
                x += d * std::cos(yaw);
                y += d * std::sin(yaw);
            } else {
                double R = params.wheel_base / std::tan(steer_adjusted);
                double beta = d / R;
                x += R * (std::sin(yaw + beta) - std::sin(yaw));
                y += R * (std::cos(yaw) - std::cos(yaw + beta));
                yaw += beta;
            }
        }
        double t = current->t + params.time_step;
        if (t > params.max_time) return nullptr;
        double cost = current->cost + additional_cost + distance;
        return new_node(x, y, yaw, t, cost, current, direction);
    }

    bool check_collision_along_path(Node* current, Node* new_node, std::pair<int, double> prim) {
        int direction = prim.first;
        double steer = prim.second;
        double distance = std::hypot(new_node->x - current->x, new_node->y - current->y);
        double steer_adjusted = (direction > 0) ? steer : -steer;
        double time_inc = params.time_step;
        double front_inf = params.robot_front_length * params.inflation;
        double rear_inf = params.robot_rear_length * params.inflation;
        double width_inf = params.robot_width * params.inflation;
        double diag_r = std::sqrt((front_inf + rear_inf) * (front_inf + rear_inf) + width_inf * width_inf) / 2;
        for (int i = 1; i <= params.collision_steps; ++i) {
            double frac = static_cast<double>(i) / params.collision_steps;
            double d = frac * distance;
            double x_i = current->x, y_i = current->y, yaw_i = current->yaw;
            if (direction != 0) {
                if (std::abs(steer_adjusted) < 1e-5) {
                    x_i += (d / distance) * (new_node->x - current->x);
                    y_i += (d / distance) * (new_node->y - current->y);
                } else {
                    double R = params.wheel_base / std::tan(steer_adjusted);
                    double beta = (direction * d) / R;
                    x_i += R * (std::sin(yaw_i + beta) - std::sin(yaw_i));
                    y_i += R * (std::cos(yaw_i) - std::cos(yaw_i + beta));
                    yaw_i += beta;
                }
            }
            double t_i = current->t + frac * time_inc;
            auto corners_r = get_corners(x_i, y_i, yaw_i, front_inf, rear_inf, width_inf);
            if (!is_in_bounds(corners_r, params.min_x, params.max_x, params.min_y, params.max_y)) return false;
            for (size_t idx = 0; idx < obstacles.size(); ++idx) {
                const auto& ob = obstacles[idx];
                auto [x_o, y_o, yaw_o, is_present] = get_ob_pose(ob, t_i);
                if (!is_present) continue;
                double dist = std::hypot(x_i - x_o, y_i - y_o);
                if (dist > diag_r + ob_diags[idx] + params.safety_margin) continue;
                auto corners_o = get_corners(x_o, y_o, yaw_o, ob.front, ob.rear, ob.width);
                if (rectangles_intersect(corners_r, corners_o)) return false;
            }
        }
        return true;
    }

    bool check_collision_at(Node* node) {
        double t_i = node->t;
        double front_inf = params.robot_front_length * params.inflation;
        double rear_inf = params.robot_rear_length * params.inflation;
        double width_inf = params.robot_width * params.inflation;
        double diag_r = std::sqrt((front_inf + rear_inf) * (front_inf + rear_inf) + width_inf * width_inf) / 2;
        auto corners_r = get_corners(node->x, node->y, node->yaw, front_inf, rear_inf, width_inf);
        if (!is_in_bounds(corners_r, params.min_x, params.max_x, params.min_y, params.max_y)) return false;
        for (size_t idx = 0; idx < obstacles.size(); ++idx) {
            const auto& ob = obstacles[idx];
            auto [x_o, y_o, yaw_o, is_present] = get_ob_pose(ob, t_i);
            if (!is_present) continue;
            double dist = std::hypot(node->x - x_o, node->y - y_o);
            if (dist > diag_r + ob_diags[idx] + params.safety_margin) continue;
            auto corners_o = get_corners(x_o, y_o, yaw_o, ob.front, ob.rear, ob.width);
            if (rectangles_intersect(corners_r, corners_o)) return false;
        }
        return true;
    }

    std::tuple<std::vector<std::tuple<double, double, double>>, double> analytic_expand(Node* current) {
        double dx = goal->x - current->x;
        double dy = goal->y - current->y;
        if (std::hypot(dx, dy) > params.analytic_threshold) return {{}, 0.0};
        auto [x_rs, y_rs, yaw_rs, ctypes, lengths] = ReedsShepp::path_planning(
            current->x, current->y, current->yaw, goal->x, goal->y, goal->yaw, params.max_curvature, params.rs_step_size
            );
        if (x_rs.empty()) return {{}, 0.0};
        std::vector<std::tuple<double, double, double>> rs_path;
        for (size_t i = 0; i < x_rs.size(); ++i) {
            rs_path.emplace_back(x_rs[i], y_rs[i], yaw_rs[i]);
        }
        double rs_length = 0.0;
        for (double l : lengths) rs_length += std::abs(l);
        return {rs_path, rs_length};
    }

    bool check_collision_along_rs(const std::vector<std::tuple<double, double, double>>& rs_path, double start_t) {
        double current_t = start_t;
        double front_inf = params.robot_front_length * params.inflation;
        double rear_inf = params.robot_rear_length * params.inflation;
        double width_inf = params.robot_width * params.inflation;
        double diag_r = std::sqrt((front_inf + rear_inf) * (front_inf + rear_inf) + width_inf * width_inf) / 2;
        for (size_t i = 0; i < rs_path.size() - 1; ++i) {
            auto [x1, y1, yaw1] = rs_path[i];
            auto [x2, y2, yaw2] = rs_path[i + 1];
            double dx = x2 - x1;
            double dy = y2 - y1;
            double dist = std::hypot(dx, dy);
            if (dist > 0) {
                double time_inc = dist / params.speed;
                for (int j = 1; j <= params.collision_steps; ++j) {
                    double frac = static_cast<double>(j) / params.collision_steps;
                    double x_i = x1 + frac * dx;
                    double y_i = y1 + frac * dy;
                    double yaw_i = yaw1 + frac * (yaw2 - yaw1);
                    double t_i = current_t + frac * time_inc;
                    auto corners_r = get_corners(x_i, y_i, yaw_i, front_inf, rear_inf, width_inf);
                    if (!is_in_bounds(corners_r, params.min_x, params.max_x, params.min_y, params.max_y)) return false;
                    for (size_t idx = 0; idx < obstacles.size(); ++idx) {
                        const auto& ob = obstacles[idx];
                        auto [x_o, y_o, yaw_o, is_present] = get_ob_pose(ob, t_i);
                        if (!is_present) continue;
                        double dist_o = std::hypot(x_i - x_o, y_i - y_o);
                        if (dist_o > diag_r + ob_diags[idx] + params.safety_margin) continue;
                        auto corners_o = get_corners(x_o, y_o, yaw_o, ob.front, ob.rear, ob.width);
                        if (rectangles_intersect(corners_r, corners_o)) return false;
                    }
                }
                current_t += time_inc;
            }
        }
        return true;
    }

    double calc_f(Node* node) {
        return node->cost + calc_heuristic(node);
    }

    double calc_heuristic(Node* node) {
        auto [_, __, ___, ____, lengths] = ReedsShepp::path_planning(
            node->x, node->y, node->yaw, goal->x, goal->y, goal->yaw, params.max_curvature, params.rs_step_size * 5
            );
        if (lengths.empty()) return std::numeric_limits<double>::infinity();
        double h = 0.0;
        for (double l : lengths) h += std::abs(l);
        return h;
    }

    std::vector<std::tuple<double, double, double, double>> extract_path(Node* node, const std::vector<std::tuple<double, double, double>>& rs_path) {
        std::vector<std::tuple<double, double, double, double>> timed_path;
        Node* current = node;
        while (current) {
            timed_path.emplace_back(current->t, current->x, current->y, current->yaw);
            current = current->parent;
        }
        std::reverse(timed_path.begin(), timed_path.end());
        if (!rs_path.empty()) {
            double rs_t = std::get<0>(timed_path.back());
            for (size_t i = 1; i < rs_path.size(); ++i) {
                auto [x_prev, y_prev, yaw_prev] = rs_path[i - 1];
                auto [x, y, yaw] = rs_path[i];
                double dx = x - x_prev;
                double dy = y - y_prev;
                double dist = std::hypot(dx, dy);
                double dt = dist / params.speed;
                rs_t += dt;
                timed_path.emplace_back(rs_t, x, y, yaw);
            }
        }
        return timed_path;
    }

    Node* new_node(double x, double y, double yaw, double t, double cost, Node* parent, int direction) {
        all_nodes.emplace_back(std::make_unique<Node>(x, y, yaw, t, cost, parent, direction));
        return all_nodes.back().get();
    }


public:
    PHAStar(double start_x, double start_y, double start_yaw, double goal_x, double goal_y, double goal_yaw, double start_time, double planning_speed, const std::vector<DynamicObstacle>& obs, Params p)
        : params(p), obstacles(obs) {
        params.speed = planning_speed;
        params.movement_length = params.speed * params.time_step;
        start = std::make_unique<Node>(start_x, start_y, start_yaw, start_time, 0.0, nullptr, 1);
        goal = std::make_unique<Node>(goal_x, goal_y, goal_yaw, 0.0, 0.0, nullptr, 1);
        x_width = static_cast<int>((params.max_x - params.min_x) / params.xy_resolution) + 1;
        y_width = static_cast<int>((params.max_y - params.min_y) / params.xy_resolution) + 1;
        theta_width = static_cast<int>(2 * M_PI / params.yaw_resolution);
        time_width = static_cast<int>(params.max_time / params.time_resolution) + 1;
        ob_diags.reserve(obstacles.size());
        for (const auto& ob : obstacles) {
            ob_diags.push_back(std::sqrt((ob.front + ob.rear) * (ob.front + ob.rear) + ob.width * ob.width) / 2);
        }
    }


    std::vector<std::tuple<double, double, double, double>> planning() {
        using PQElem = std::tuple<double, uint64_t, Node*>;
        auto cmp = [](const PQElem& a, const PQElem& b) { return std::get<0>(a) > std::get<0>(b) || (std::get<0>(a) == std::get<0>(b) && std::get<1>(a) > std::get<1>(b)); };
        std::priority_queue<PQElem, std::vector<PQElem>, decltype(cmp)> open_set(cmp);
        uint64_t node_id = 0;
        size_t start_id = calc_grid_index(start.get());
        std::unordered_map<size_t, double> g_costs;
        g_costs[start_id] = 0.0;
        open_set.emplace(calc_f(start.get()), node_id++, start.get());
        std::unordered_set<size_t> closed_set;

        while (!open_set.empty()) {
            auto [f, _, current] = open_set.top();
            open_set.pop();
            size_t n_id = calc_grid_index(current);
            if (closed_set.count(n_id)) continue;
            if (current->cost > g_costs[n_id]) continue;  // Outdated entry
            closed_set.insert(n_id);

            if (!check_collision_at(current)) continue;

            auto [rs_path, rs_length] = analytic_expand(current);
            if (!rs_path.empty()) {
                if (check_collision_along_rs(rs_path, current->t)) {
                    std::cout << "Goal found with analytic expansion!" << std::endl;
                    return extract_path(current, rs_path);
                }
            }

            for (auto prim : params.motion_primitives) {
                Node* new_node = generate_node(current, prim);
                if (!new_node) continue;
                if (!check_collision_along_path(current, new_node, prim)) continue;
                size_t new_id = calc_grid_index(new_node);
                if (closed_set.count(new_id)) continue;
                double new_g = new_node->cost;
                auto it = g_costs.find(new_id);
                if (it != g_costs.end() && new_g >= it->second) continue;  // Worse or equal
                g_costs[new_id] = new_g;
                open_set.emplace(new_g + calc_heuristic(new_node), node_id++, new_node);
            }
        }

        std::cout << "No path found" << std::endl;
        return {};
    }
};


#endif // PHASTAR_H
