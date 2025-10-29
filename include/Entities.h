#ifndef ENTITIES_H
#define ENTITIES_H

#include <string>
#include <vector>

#include <Point.h>

enum class EntityType { ROBOT, OBJECT };

struct Pose : public Point {
    double yaw = 0.0;
};

struct OccuRect {
    double front_length = 0.0;
    double rear_length = 0.0;
    double width = 0.0;
};

struct EntityMeta {
    std::string name;
    EntityType type;
    Pose initial_pose;
    OccuRect size;
    virtual ~EntityMeta() = default;  // Added to make polymorphic
};

struct RobotMeta : public EntityMeta {
    double min_turning_radius = 0.0;
    double wheel_base = 0.0;
    double speed_transit = 0.0;
    double speed_transfer = 0.0;
};

struct ObjectMeta : public EntityMeta {
    Pose goal_pose;
};

struct Waypoint : public Pose {
    double time = 0.0;
    double linear_velocity = 0.0;
    double steering_angle = 0.0;
};

struct Trajectory {
    EntityMeta* entity = nullptr;
    double start_time = 0.0;
    std::vector<Waypoint> waypoints;
    bool is_transfer = false;
    EntityMeta* transferred_object = nullptr;
};

#endif // ENTITIES_H
