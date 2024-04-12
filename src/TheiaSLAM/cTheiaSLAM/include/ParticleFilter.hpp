#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <unordered_map>
#include <vector>
#include <memory>
#include <string>
#include <random>

#include "TraversalGraph.hpp"
#include "Particle.hpp"
#include "Pose.hpp"
#include "Trajectory.hpp"
#include "TraversalEdge.hpp"

class ParticleFilter {
public:
    std::shared_ptr<TraversalGraph> graph;
    std::vector<std::shared_ptr<Particle>> particles;
    std::shared_ptr<TraversalEdge> edge;
    double last_update = 0;
    double draw_size = 20;
    std::array<double, 4> draw_color = {1, 1, 0, .1};
    double spawn_rate;
    double avg_speed;
    double dev_speed;
    double avg_rot;
    double dev_rot;
    std::string behavior;
    int num_particles;
    double position_variance;
    double angle_variance;
    double update_rate;
    std::uniform_real_distribution<double> rand_prob = std::uniform_real_distribution<double>(0.0,1.0);
    std::vector<double> likelihoods;
    std::normal_distribution<double> unit_normal = std::normal_distribution<double>(0.0, 1.0);
    std::mt19937 rng = std::mt19937(std::random_device{}());
    ParticleFilter(std::unordered_map<std::string, std::string> dict, std::shared_ptr<TraversalGraph> fgraph, std::shared_ptr<Pose> pose, double time);
    void initialize_particles(const std::shared_ptr<Pose> pose, double time);
    void clear_visible_particles(const std::shared_ptr<Pose> pose, double time);
    void update_particles(const std::shared_ptr<Pose> pose, double time);
    void propogate_particles(double prop_time);
    void trim_history(double trim_time);
    double rand_speed();
    double rand_rot();
    double calculate_cost(const std::shared_ptr<Trajectory> trajectory);
    void normalize_likelihoods();
    static double normal_pdf(double x, double mean = 0, double stddev = 1);
};

#endif // PARTICLE_FILTER_HPP
