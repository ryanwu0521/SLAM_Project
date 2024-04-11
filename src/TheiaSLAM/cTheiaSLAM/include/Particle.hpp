#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>
#include <memory>
#include <string>
#include "TraversalGraph.hpp" // Include the header file for Trajectory class
#include "Pose.hpp"
#include "Trajectory.hpp"
class Particle {
public:
    Particle(Pose pose, const std::shared_ptr<TraversalEdge> edge, const std::shared_ptr<TraversalNode> next_node, const std::shared_ptr<TraversalGraph> fgraph, double speed, double rot, const std::string behavior, double time);

    bool collision_check(const std::shared_ptr<Trajectory> check_trajectory);
    void add_next_traj();
    void trim_history(double time);
    Pose get_pose_at_time(double time);
    std::shared_ptr<TraversalEdge> get_edge_at_time(double time);
    std::shared_ptr<TraversalNode> get_next_node_at_time(double time);


    void _forward_behavior();
    void _random_behavior();
    void propogate(double time);
    std::string behavior;
    double linear_speed;
    double angular_speed;
    std::shared_ptr<TraversalGraph> graph;
    std::vector<std::shared_ptr<Trajectory>> trajectory;
};

#endif // PARTICLE_HPP
