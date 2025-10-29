/*************************************
 * Prioritized Hybrid Astar Demo
 *
 * 2025.10.24
 * Jeeho Ahn, jeeho@umich.edu
*************************************/

#include <Reeds_Shepp.h>
#include <Visualization.h>

#include <PHAstar.h>

int main(int argc, char** argv) {
    Params params;
    params.analytic_threshold = 5.0 * params.max_steer;  // Adjust if needed

    RobotMeta robot1;
    robot1.name = "robot1";
    robot1.type = EntityType::ROBOT;
    robot1.initial_pose = {1.0, 1.0, 0.0};
    robot1.size = {0.4, 0.2, 0.3};
    robot1.min_turning_radius = 1.0;  // Example
    robot1.wheel_base = 0.4;
    robot1.speed_transit = 0.2;
    robot1.speed_transfer = 0.15;

    RobotMeta robot2;
    robot2.name = "robot2";
    robot2.type = EntityType::ROBOT;
    robot2.initial_pose = {1.0, 2.0, 0.0};
    robot2.size = {0.4, 0.2, 0.3};
    robot2.min_turning_radius = 1.0;
    robot2.wheel_base = 0.4;
    robot2.speed_transit = 0.2;
    robot2.speed_transfer = 0.15;

    ObjectMeta obj1;
    obj1.name = "obj1";
    obj1.type = EntityType::OBJECT;
    obj1.initial_pose = {1.6, 1.0, 0.0};  // In front of robot1
    obj1.size = {0.075, 0.075, 0.15};
    obj1.goal_pose = {4.6, 4.0, M_PI / 2};

    ObjectMeta obj2;
    obj2.name = "obj2";
    obj2.type = EntityType::OBJECT;
    obj2.initial_pose = {3.5, 3.0, 0.0};
    obj2.size = {0.075, 0.075, 0.15};
    obj2.goal_pose = {3.6, 3.0, M_PI / 2};

    std::unordered_map<std::string, EntityMeta*> entities = {
        {"robot1", &robot1}, {"robot2", &robot2}, {"obj1", &obj1}, {"obj2", &obj2}
    };

    TimeTable timetable(0.5);
    timetable.add_initial(entities);

    std::vector<Trajectory> all_trajectories;

    // Priority order
    std::vector<std::string> priority = {"robot1", "robot2"};

    // Plans: robot name -> {goal, is_transfer, obj_name}
    std::unordered_map<std::string, std::tuple<Pose, bool, std::string>> robot_plans = {
        {"robot1", {{4.0, 4.0, M_PI / 2}, false, "obj1"}},
        {"robot2", {{3.0, 1.0, 1.7}, false, ""}} // todo: problem found when goal orientation is pi/2
    };

    for (const auto& r_name : priority) {
        auto [goal_pose, trans, obj_name] = robot_plans[r_name];
        RobotMeta* r = dynamic_cast<RobotMeta*>(entities[r_name]);
        PHAStar planner(r, goal_pose, &timetable, &entities, params, trans, obj_name);
        auto start_time = std::chrono::high_resolution_clock::now();
        auto waypoints = planner.planning();
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> planning_time = end_time - start_time;
        std::cout << "Planning time for " << r_name << ": " << planning_time.count() << " seconds" << std::endl;

        if (!waypoints.empty()) {
            std::cout << "Last waypoint for " << r_name << ": time=" << waypoints.back().time << ", x=" << waypoints.back().x << ", y=" << waypoints.back().y << ", yaw=" << waypoints.back().yaw << std::endl;
            Trajectory traj;
            traj.entity = r;
            traj.start_time = 0.0;
            traj.waypoints = waypoints;
            traj.is_transfer = trans;
            traj.transferred_object = trans ? entities[obj_name] : nullptr;
            all_trajectories.push_back(traj);
            timetable.add_trajectory(traj);
        } else {
            std::cout << "Failed to find a path for " << r_name << std::endl;
        }
    }

    QApplication app(argc, argv);
    QMainWindow win;
    VizWidget* viz = new VizWidget(timetable, entities, all_trajectories, params);
    win.setCentralWidget(viz);

    QWidget* panel = new QWidget;
    QHBoxLayout* layout = new QHBoxLayout(panel);
    QSlider* slider = new QSlider(Qt::Horizontal);
    double max_t = 0.0;
    for (const auto& traj : all_trajectories) {
        if (!traj.waypoints.empty()) max_t = std::max(max_t, traj.waypoints.back().time);
    }
    slider->setRange(0, static_cast<int>(max_t * 100));
    layout->addWidget(slider);
    QLabel* timeLabel = new QLabel("Time: 0.00 s");
    layout->addWidget(timeLabel);

    QObject::connect(slider, &QSlider::valueChanged, [viz, timeLabel](int val) {
        double t = val / 100.0;
        viz->setTime(t);
        timeLabel->setText(QString("Time: %1 s").arg(t, 0, 'f', 2));
    });

    QDockWidget* dock = new QDockWidget;
    dock->setWidget(panel);
    win.setWindowTitle("Prioritized Hybrid A* Demo - Jeeho Ahn");
    win.addDockWidget(Qt::BottomDockWidgetArea, dock);
    win.resize(600, 600);
    win.show();
    return app.exec();
}
