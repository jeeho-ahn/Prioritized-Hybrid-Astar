#ifndef PARAMS_H
#define PARAMS_H

struct Params {
    double xy_resolution = 0.1;
    double yaw_resolution = M_PI / 6.0; // 30 degrees
    double time_step = 2.0;
    double time_resolution = time_step;
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 5.0;
    double max_y = 5.0;
    double robot_front_length = 0.4;
    double robot_rear_length = 0.2;
    double robot_width = 0.3;
    double wheel_base = robot_front_length;
    double max_steer = M_PI / 6.0; // 30 degrees
    double min_turn_radius = wheel_base / std::tan(max_steer);
    double max_curvature = 1.0 / min_turn_radius;
    double speed = 0.2;
    double movement_length = speed * time_step;
    std::vector<std::pair<int, double>> motion_primitives = {
        {1, 0.0}, {1, max_steer}, {1, -max_steer}, {1, max_steer / 2}, {1, -max_steer / 2},
        {-1, 0.0}, {-1, max_steer}, {-1, -max_steer}, {-1, max_steer / 2}, {-1, -max_steer / 2},
        {0, 0.0}
    };
    double turn_penalty = 1.25;
    double reverse_penalty = 50.0;
    double switch_penalty = 1.2;
    double wait_penalty = 0.05;
    double max_time = 100.0;
    int collision_steps = 3;
    double analytic_threshold = 5.0 * min_turn_radius;
    double rs_step_size = 0.2;
    double inflation = 1.1;
    double safety_margin = 0.05;
};

#endif // PARAMS_H
