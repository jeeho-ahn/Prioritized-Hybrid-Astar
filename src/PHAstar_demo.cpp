/*************************************
 * Prioritized Hybrid Astar Demo
 *
 * 2025.10.24
 * Jeeho Ahn, jeeho@umich.edu
*************************************/

#include <Reeds_Shepp.h>
#include <Visualization.h>

#include <PHAstar.h>
#include <iomanip>

const bool print_path = false;

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
    obj1.initial_pose = {1.6, 1.0, 0};
    obj1.size = {0.075, 0.075, 0.15};
    obj1.goal_pose = {4.6, 4.0, M_PI/2};

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

    std::unordered_map<std::string, double> robot_arrival_times;  // Track last arrival per robot for chaining

    // Plans: robot name -> {goal, is_transfer, obj_name}
    // Single list of plans: vector<std::tuple<std::string, Pose, bool, std::string, double>>  // robot_name, goal_pose, is_transfer, obj_name, placeholder_start_time (overridden dynamically)
    std::vector<std::tuple<std::string, Pose, bool, std::string, double>> robot_plans = {
        {"robot1", {4.0, 4.0, M_PI / 2}, true, "obj1", 0.0},
        {"robot2", {4.5, 1.0, M_PI / 2}, false, "", 0.0},
        {"robot2", {2.0, 3.0, 0.0}, false, "", 39.0}
    };


    std::unordered_map<std::string, double> robot_current_times;  // Track last arrival per robot for chaining
    std::unordered_map<std::string, Pose> robot_current_poses;  // Track last pose per robot

    for (auto& plan : robot_plans) {
        auto [r_name, goal_pose, trans, obj_name, provided_start_t] = plan;
        RobotMeta* r = dynamic_cast<RobotMeta*>(entities[r_name]);
        double current_start_t = (provided_start_t > 0.0) ? provided_start_t : robot_current_times[r_name];  // Use provided if >0, else dynamic
        Pose start_pose = robot_current_poses.count(r_name) ? robot_current_poses[r_name] : r->initial_pose;
        r->initial_pose = start_pose;  // Temporarily set for planner

        PHAStar planner(r, goal_pose, &timetable, &entities, params, trans, obj_name, current_start_t);
        auto start_time = std::chrono::high_resolution_clock::now();
        auto waypoints = planner.planning();
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> planning_time = end_time - start_time;
        std::cout << "Planning time for " << r_name << ": " << planning_time.count() << " seconds" << std::endl;

        if (!waypoints.empty()) {
            std::cout << "Last waypoint for " << r_name << ": time=" << waypoints.back().time << ", x=" << waypoints.back().x << ", y=" << waypoints.back().y << ", yaw=" << waypoints.back().yaw << std::endl;

            if(print_path){
                for (const auto& wp : waypoints) {
                    std::cout << "time=" << wp.time << ", x=" << wp.x << ", y=" << wp.y << ", yaw=" << wp.yaw << std::endl;
                }
            }

            Trajectory traj;
            traj.entity = r;
            traj.start_time = current_start_t;
            traj.waypoints = waypoints;
            traj.is_transfer = trans;
            traj.transferred_object = trans ? entities[obj_name] : nullptr;
            all_trajectories.push_back(traj);
            timetable.add_trajectory(traj);

            // Update for potential next plan for this robot
            robot_current_times[r_name] = waypoints.back().time;
            robot_current_poses[r_name] = {waypoints.back().x, waypoints.back().y, waypoints.back().yaw};
        } else {
            std::cout << "Failed to find a path for " << r_name << std::endl;
            // Continue to next plan, but note failure
        }
    }



    // Print TimeTable poses for robot2

    if(print_path)
    {
        std::cout << "TimeTable poses for robot2:" << std::endl;
        EntityMeta* robot2_ent = entities["robot2"];
        double max_tt = 0.0;
        for (const auto& traj : all_trajectories) {
            if (!traj.waypoints.empty()) {
                max_tt = std::max(max_tt, traj.waypoints.back().time);
            }
        }
        for (double t = 0.0; t <= max_tt + 1e-6; t += 0.1) {  // Step of 0.1 for fine-grained view
            Pose p = timetable.get_pose(robot2_ent, t);
            std::cout << "t=" << std::fixed << std::setprecision(4) << t
                      << ", x=" << p.x << ", y=" << p.y << ", yaw=" << p.yaw << std::endl;
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
