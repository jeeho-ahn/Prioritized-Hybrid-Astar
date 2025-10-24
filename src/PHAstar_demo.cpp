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
    Obstacle static_ob = {2.0, 2.0, 0.0, 0.25, 0.25, 0.5, 0.0, 0.0, 0.0};
    Obstacle dynamic_ob = {2.5, 0.9, M_PI, 0.4, 0.2, 0.3, -0.1, 0.0, 0.0};
    Obstacle dynamic_ob2 = {2.5, 1.4, M_PI * 82 / 180, 0.4, 0.2, 0.3, 0.0451, 0.062, 0.0};
    std::vector<Obstacle> obstacles = {static_ob, dynamic_ob, dynamic_ob2};
    double start[3] = {1.0, 1.0, 0.0};
    double goal[3] = {4.0, 4.0, M_PI / 2};

    PHAStar planner(start[0], start[1], start[2], goal[0], goal[1], goal[2], obstacles, params);

    auto start_time = std::chrono::high_resolution_clock::now();
    auto timed_path = planner.planning();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time = end_time - start_time;
    std::cout << "Planning time: " << planning_time.count() << " seconds" << std::endl;

    if (!timed_path.empty()) {
        std::cout << "Resulting timed path (t, x, y, yaw):" << std::endl;
        for (const auto& step : timed_path) {
            std::cout << std::get<0>(step) << ", " << std::get<1>(step) << ", " << std::get<2>(step) << ", " << std::get<3>(step) << std::endl;
        }

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
        win.setWindowTitle("Prioritized Hybrid A* Demo - Jeeho Ahn");
        win.addDockWidget(Qt::BottomDockWidgetArea, dock);
        win.resize(600, 600);
        win.show();
        return app.exec();
    } else {
        std::cout << "Failed to find a path." << std::endl;
    }
    return 0;
}
