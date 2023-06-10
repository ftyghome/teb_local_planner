#include "teb_local_planner/homotopy_class_planner.h"
#include <rclcpp/rclcpp.hpp>
#include <random>

class OptimalPlannerTest : public teb_local_planner::TebOptimalPlanner {
public:

    void SetUp(const nav2_util::LifecycleNode::SharedPtr &node) {
        teb_local_planner::RobotFootprintModelPtr robot_model;
        teb_local_planner::TebVisualizationPtr visualization;
        std::default_random_engine eng(114);
        std::uniform_real_distribution<double> x1(3.0, 6.0);
        std::uniform_real_distribution<double> y1(3.0, 6.7);

        std::uniform_real_distribution<double> x2(2.1, 6.0);
        std::uniform_real_distribution<double> y2(2.1, 6.6);

        robot_model.reset(new teb_local_planner::CircularRobotFootprint(0.5));

        for (int i = 0; i < 5; i++) {
            obstacles.push_back(
                    teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(x1(eng), y1(eng)))
            );
            obstacles.push_back(
                    teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(x2(eng), y2(eng)))
            );
        }

        visualization.reset(
                new teb_local_planner::TebVisualization(node, cfg)
        );

        cfg.hcp.visualize_hc_graph = true;
        cfg.robot_model = robot_model;
        cfg.robot.max_vel_x = 2;
        cfg.robot.max_vel_y = 0;
        cfg.robot.max_vel_theta = 1;
        cfg.robot.acc_lim_theta = 1;


        cfg.optim.weight_kinematics_forward_drive = 1000;
        cfg.trajectory.max_global_plan_lookahead_dist = 5.0;


        initialize(node, cfg, &obstacles, visualization, nullptr);
        std::cerr << "setting up" << std::endl;
    }

    teb_local_planner::ObstContainer obstacles;
    teb_local_planner::TebConfig cfg;


};

void outputPlanResult(OptimalPlannerTest &instance, const std::string &outPath) {
    std::ofstream f(outPath);
    f << "{\n"
      << "\t\"Plan\": [";
    for (const auto pose: instance.teb().poses()) {
        f << "[" << pose->x() << ", " << pose->y() << "]";
        if (pose != instance.teb().poses().back()) f << ", ";
    }
    f << "],\n";
    f << "\t\"Obstacles\": [";
    for (const auto &pose: instance.obstacles) {
        f << "[" << pose->getCentroid().x() << ", " << pose->getCentroid().y() << "]";
        if (pose != instance.obstacles.back()) f << ", ";
    }
    f << "]\n";

    f << "}";
    f.close();
}

void test() {
    OptimalPlannerTest test;
    nav2_util::LifecycleNode::SharedPtr node(new nav2_util::LifecycleNode("test"));
    test.SetUp(node);

    using namespace teb_local_planner;
    PoseSE2 start(0, 4, 0);
    PoseSE2 goal(10, 4, 0);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;

    std::vector<geometry_msgs::msg::PoseStamped> initial_plan;

    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::PoseStamped goal_pose;

    start.toPoseMsg(start_pose.pose);
    goal.toPoseMsg(goal_pose.pose);

    initial_plan.push_back(start_pose);
    initial_plan.push_back(goal_pose);

    std::cerr << "before plan" << std::endl;

    std::cerr << test.plan(initial_plan, &twist, false) << std::endl;

    std::cerr << "after plan" << std::endl;

    outputPlanResult(test, "out.json");


}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    test();
}