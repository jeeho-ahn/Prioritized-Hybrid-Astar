/*************************************
 * Prioritized Hybrid Astar Demo
 *
 * 2025.10.24
 * Jeeho Ahn, jeeho@umich.edu
*************************************/

#include <Reeds_Shepp.h>
#include <Visualization.h>

#include <PHAstar.h>

int visualize(const std::vector<std::tuple<double, double, double, double>>& timed_path, const std::vector<DynamicObstacle>& obstacles, const Params& params, int argc, char** argv) {
    QApplication app(argc, argv);
    QMainWindow win;
    VizWidget* viz = new VizWidget(timed_path, obstacles, params);
    win.setCentralWidget(viz);

    QWidget* panel = new QWidget;
    QHBoxLayout* layout = new QHBoxLayout(panel);
    QSlider* slider = new QSlider(Qt::Horizontal);
    double max_t = std::get<0>(timed_path.back());
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
    win.addDockWidget(Qt::BottomDockWidgetArea, dock);
    win.resize(600, 600);
    viz->setTime(0.0);  // Explicitly set initial time
    win.show();
    return app.exec();
}


int main(int argc, char** argv) {
    Params params;
    DynamicObstacle static_ob;
    static_ob.name = "static";
    static_ob.start_time = 0.0;
    static_ob.front = 0.25;
    static_ob.rear = 0.25;
    static_ob.width = 0.5;
    // Present from t=0 to t=10, removed after t=10, added back at t=20 at different position
    static_ob.path = {
        {0.0, 0.0, 0.0, 3.0, 2.5, 0.0, true},
        {13.0, 0.0, 0.0, 3.0, 2.5, 0.0, true},
        {13.0001, 0.0, 0.0, 0.0, 0.0, 0.0, false}, // Removed
        {18.0, 0.0, 0.0, 3.0, 3.0, M_PI / 4, true} // Added back at new position
    };
    std::vector<DynamicObstacle> obstacles = {};
    // Robot 1 (higher priority) first path
    double robot1_start_time1 = 0.0;
    double robot1_speed1 = 0.2;
    double robot1_start1[3] = {1.0, 1.0, 0.0};
    double robot1_goal1[3] = {3.0, 2.5, M_PI / 2};
    PHAStar planner1(robot1_start1[0], robot1_start1[1], robot1_start1[2], robot1_goal1[0], robot1_goal1[1], robot1_goal1[2], robot1_start_time1, robot1_speed1, obstacles, params);
    auto start_time1 = std::chrono::high_resolution_clock::now();
    auto robot1_timed_path1 = planner1.planning();
    auto end_time1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time1 = end_time1 - start_time1;
    std::cout << "Robot 1 First Path Planning time: " << planning_time1.count() << " seconds" << std::endl;
    if (robot1_timed_path1.empty()) {
        std::cout << "Robot 1 failed to find first path." << std::endl;
        return 1;
    }
    std::cout << "Robot 1 First timed path (t, x, y, yaw):" << std::endl;
    for (const auto& step : robot1_timed_path1) {
        std::cout << std::get<0>(step) << ", " << std::get<1>(step) << ", " << std::get<2>(step) << ", " << std::get<3>(step) << std::endl;
    }
    double robot1_arrival_time1 = std::get<0>(robot1_timed_path1.back());
    // Convert robot1's first path to DynamicObstacle
    DynamicObstacle robot1_as_ob1;
    robot1_as_ob1.name = "robot1_path1";
    robot1_as_ob1.start_time = robot1_start_time1;
    robot1_as_ob1.front = params.robot_front_length;
    robot1_as_ob1.rear = params.robot_rear_length;
    robot1_as_ob1.width = params.robot_width;
    for (const auto& step : robot1_timed_path1) {
        double time = std::get<0>(step);
        double x = std::get<1>(step);
        double y = std::get<2>(step);
        double yaw = std::get<3>(step);
        double vx = robot1_speed1;
        double steer = 0.0;
        robot1_as_ob1.path.emplace_back(time, vx, steer, x, y, yaw, true);
    }
    // Add removal waypoint after arrival
    robot1_as_ob1.path.emplace_back(robot1_arrival_time1 + 0.1, 0.0, 0.0, std::get<1>(robot1_timed_path1.back()), std::get<2>(robot1_timed_path1.back()), std::get<3>(robot1_timed_path1.back()), false);
    obstacles.push_back(robot1_as_ob1);
    // Robot 2 planning
    double robot2_start_time = 0.0;
    double robot2_speed = 0.15;
    double robot2_start[3] = {0.5, 0.5, 0.0};
    double robot2_goal[3] = {2.0, 3.5, 0};
    PHAStar planner2(robot2_start[0], robot2_start[1], robot2_start[2], robot2_goal[0], robot2_goal[1], robot2_goal[2], robot2_start_time, robot2_speed, obstacles, params);
    auto start_time2 = std::chrono::high_resolution_clock::now();
    auto robot2_timed_path = planner2.planning();
    auto end_time2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time2 = end_time2 - start_time2;
    std::cout << "Robot 2 Planning time: " << planning_time2.count() << " seconds" << std::endl;
    if (robot2_timed_path.empty()) {
        std::cout << "Robot 2 failed to find a path." << std::endl;
        return 1;
    }
    std::cout << "Robot 2 Resulting timed path (t, x, y, yaw):" << std::endl;
    for (const auto& step : robot2_timed_path) {
        std::cout << std::get<0>(step) << ", " << std::get<1>(step) << ", " << std::get<2>(step) << ", " << std::get<3>(step) << std::endl;
    }
    double robot2_arrival_time = std::get<0>(robot2_timed_path.back());
    // Convert robot2's path to DynamicObstacle
    DynamicObstacle robot2_as_ob;
    robot2_as_ob.name = "robot2";
    robot2_as_ob.start_time = robot2_start_time;
    robot2_as_ob.front = params.robot_front_length;
    robot2_as_ob.rear = params.robot_rear_length;
    robot2_as_ob.width = params.robot_width;
    for (const auto& step : robot2_timed_path) {
        double time = std::get<0>(step);
        double x = std::get<1>(step);
        double y = std::get<2>(step);
        double yaw = std::get<3>(step);
        double vx = robot2_speed;
        double steer = 0.0;
        robot2_as_ob.path.emplace_back(time, vx, steer, x, y, yaw, true);
    }
    obstacles.push_back(robot2_as_ob);
    // Robot 1 second path
    double robot1_start_time2 = robot1_arrival_time1+0.1;
    double robot1_speed2 = 0.18;
    double robot1_start2[3] = {robot1_goal1[0], robot1_goal1[1], robot1_goal1[2]};
    double robot1_goal2[3] = {3.0, 3.5, M_PI / 2}; // Slightly different to allow planning
    PHAStar planner1_second(robot1_start2[0], robot1_start2[1], robot1_start2[2], robot1_goal2[0], robot1_goal2[1], robot1_goal2[2], robot1_start_time2, robot1_speed2, obstacles, params);
    auto start_time3 = std::chrono::high_resolution_clock::now();
    auto robot1_timed_path2 = planner1_second.planning();
    auto end_time3 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time3 = end_time3 - start_time3;
    std::cout << "Robot 1 Second Path Planning time: " << planning_time3.count() << " seconds" << std::endl;
    if (robot1_timed_path2.empty()) {
        std::cout << "Robot 1 failed to find second path." << std::endl;
        return 1;
    }
    std::cout << "Robot 1 Second timed path (t, x, y, yaw):" << std::endl;
    for (const auto& step : robot1_timed_path2) {
        std::cout << std::get<0>(step) << ", " << std::get<1>(step) << ", " << std::get<2>(step) << ", " << std::get<3>(step) << std::endl;
    }
    // For visualization, visualize robot1's second path
    if (!robot1_timed_path2.empty()) {
        std::cout << "Visualizing Robot 1 Second Path" << std::endl;
        visualize(robot1_timed_path2, obstacles, params, argc, argv);
    }

    if (!robot2_timed_path.empty()) {
        std::cout << "Visualizing Robot 2 Path" << std::endl;
        visualize(robot2_timed_path, obstacles, params, argc, argv);
    }

    return 0;
}
