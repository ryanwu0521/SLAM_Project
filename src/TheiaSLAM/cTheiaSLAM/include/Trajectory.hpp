#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "Pose.hpp"
#include "Slerp.hpp"
#include "TraversalNode.hpp"
#include "TraversalEdge.hpp"

#include <memory>
class Trajectory {
private:
    void _calculate_times();
    void _setup_slerp();
    std::unique_ptr<Slerp> slerp;

    
public:
    Pose start_pose;
    Pose end_pose;
    double linear_speed;
    double angular_speed;
    double start_time;
    double wait_time;
    double turn_time;
    double end_time;

    std::shared_ptr<TraversalNode> next_node;
    std::shared_ptr<TraversalNode> prev_node;
    std::shared_ptr<TraversalEdge> edge;

    Trajectory(Pose start_pose, Pose end_pose, double linear_speed, double angular_speed,
               double start_time, double wait_time = 0);

    Trajectory(Pose start_pose, std::shared_ptr<TraversalNode> next_node, std::shared_ptr<TraversalEdge> edge, double linear_speed, double angular_speed, double start_time, double wait_time = 0.001);

    Pose get_pose_at_time(double time);
    bool is_finished(double time);
};

#endif // TRAJECTORY_HPP
