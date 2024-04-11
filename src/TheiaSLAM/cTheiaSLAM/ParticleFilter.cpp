#include "include/ParticleFilter.hpp"
#include <cmath>
#include <random>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


ParticleFilter::ParticleFilter(std::unordered_map<std::string, std::string> dict, std::shared_ptr<TraversalGraph> fgraph, std::shared_ptr<Pose> pose, double time){
    graph = fgraph;
    spawn_rate = std::stod(dict["spawn_rate"]);
    avg_speed = std::stod(dict["avg_speed"]);
    dev_speed = std::stod(dict["dev_speed"]);
    avg_rot = std::stod(dict["avg_rot"]);
    dev_rot = std::stod(dict["dev_rot"]);
    behavior = dict["behavior"];
    num_particles = std::stoi(dict["num_particles"]);
    position_variance = std::stod(dict["position_variance"]);
    angle_variance = std::stod(dict["angle_variance"]);
    update_rate = std::stod(dict["update_rate"]);
    initialize_particles(pose, time);
}

void ParticleFilter::initialize_particles(std::shared_ptr<Pose> pose, double time) {
    likelihoods.resize(num_particles, 1.0/num_particles);
    particles.reserve(num_particles);

    std::shared_ptr<TraversalEdge> edge = graph->get_closest_edge(pose->get_position());
    std::shared_ptr<TraversalNode> next_node;

    if (edge->node1->x == pose->get_position()[0] && edge->node1->y == pose->get_position()[1]) {
        next_node = edge->node1;
    } else if (edge->node2->x == pose->get_position()[0] && edge->node2->y == pose->get_position()[1]) {
        next_node = edge->node2;
    } else {
        double heading = pose->get_heading_from_orientation();
        double angle1 = atan2(edge->node1->y - pose->get_position()[1], edge->node1->x - pose->get_position()[0]) / M_PI * 180;
        double angle2 = atan2(edge->node2->y - pose->get_position()[1], edge->node2->x - pose->get_position()[0]) / M_PI * 180;
        double del_1 = std::abs(angle1 - heading);
        if (del_1 > 180) {
            del_1 = 360 - del_1;
        }

        double del_2 = std::abs(angle2 - heading);
        if (del_2 > 180) {
            del_2 = 360 - del_2;
        }

        double p1 = del_2 / (del_1 + del_2);

        for (int i = 0; i < num_particles; ++i) {
            double rand_float = static_cast<double>(std::rand()) / RAND_MAX;
            if (rand_float < p1) {
                next_node = edge->node1;
            } else {
                next_node = edge->node2;
            }
            particles.push_back(std::make_shared<Particle>(*pose.get(), edge, next_node, graph, rand_speed(), rand_rot(), behavior, time));
        }
        return;
    }

    for (int i = 0; i < num_particles; ++i) {
        particles.push_back(std::make_shared<Particle>(*pose.get(), edge, next_node, graph, rand_speed(), rand_rot(), behavior, time));
    }
}

void ParticleFilter::clear_visible_particles(const std::shared_ptr<Pose> pose, double time) {
    if (time - last_update > update_rate) {
        last_update = time;
    } else {
        return;
    }

    std::vector<bool> particle_seen(num_particles, false);
    double heading1 = pose->get_heading_from_orientation();

    for (int i = 0; i < num_particles; ++i) {
        Pose ppose = particles[i]->get_pose_at_time(time);
        std::array<double, 3> dposition = {ppose.get_position()[0] - pose->get_position()[0], ppose.get_position()[1] - pose->get_position()[1], 0};
        double heading2 = atan2(dposition[1], dposition[0]) / M_PI * 180;
        double heading_to_particle = std::abs(heading2 - heading1);
        if (heading_to_particle > 180) {
            heading_to_particle -= 360;
        }
        if (heading_to_particle < -180) {
            heading_to_particle += 360;
        }

        if (std::abs(heading_to_particle) < 50) {
            likelihoods[i] /= 100;
            particle_seen[i] = true;
        }
    }

    if (!std::any_of(particle_seen.begin(), particle_seen.end(), [](bool seen) { return seen; })) {
        return;
    }

    normalize_likelihoods();

    for (int i = 0; i < num_particles; ++i) {
        if (!particle_seen[i]) {
            continue;
        }

        double p_sum = 0;
        double p = rand_prob(rng);
        for (int j = 0; j < num_particles; ++j) {
            p_sum += likelihoods[j];
            if (p_sum > p) {
                particles[i] = std::make_shared<Particle>(particles[j]->get_pose_at_time(time), particles[j]->get_edge_at_time(time), particles[j]->get_next_node_at_time(time), graph, rand_speed(), rand_rot(), behavior, time);
                break;
            }
        }
    }
}

void ParticleFilter::normalize_likelihoods(){ //TODO: Make log likelihood
    double norm_likelihoods = 0;
    for (double& likelihood : likelihoods) {
        norm_likelihoods += likelihood*likelihood;
    }

    norm_likelihoods = std::sqrt(norm_likelihoods);

    for(double& likelihood : likelihoods){
        likelihood /= norm_likelihoods;
    }
}

void ParticleFilter::update_particles(std::shared_ptr<Pose> pose, double time) {
    if (time - last_update > update_rate) {
        last_update = time;
    } else {
        return;
    }

    std::vector<std::shared_ptr<Particle>> old_particles = particles;

    if (edge == nullptr || edge->get_distance_from(pose->get_position()) > 0.02) {
        edge = graph->get_closest_edge(pose->get_position());
    }

    std::shared_ptr<TraversalNode> next_node;
    double ratio = 1 - spawn_rate;
    double p1 = 0.0;

    if (edge->node1->x == pose->get_position()[0] && edge->node1->y == pose->get_position()[1]) {
        p1 = 1.0;
    } else if (edge->node2->x == pose->get_position()[0] && edge->node2->y == pose->get_position()[1]) {
        p1 = 0.0;
    } else {
        double heading = pose->get_heading_from_orientation();
        double angle1 = atan2(edge->node1->y - pose->get_position()[1], edge->node1->x - pose->get_position()[0]) / M_PI * 180;
        double angle2 = atan2(edge->node2->y - pose->get_position()[1], edge->node2->x - pose->get_position()[0]) / M_PI * 180;
        double del_1 = std::abs(angle1 - heading);
        if (del_1 > 180) {
            del_1 = 360 - del_1;
        }

        double del_2 = std::abs(angle2 - heading);
        if (del_2 > 180) {
            del_2 = 360 - del_2;
        }
        p1 = del_2 / (del_1 + del_2);
    }

    for (int i = 0; i < num_particles; ++i) {
        Pose ppose = particles[i]->get_pose_at_time(time);

        double z_pos = std::sqrt(std::pow(ppose.get_position()[0] - pose->get_position()[0], 2) + std::pow(ppose.get_position()[1] - pose->get_position()[1], 2)) / position_variance;
        double d_rot = std::abs(ppose.get_heading_from_orientation() - pose->get_heading_from_orientation());
        if (d_rot > 180) {
            d_rot = 360 - d_rot;
        }
        double z_rot = d_rot / angle_variance;

        likelihoods[i] = normal_pdf(z_pos) * normal_pdf(z_rot); //TODO: Double check this is correct
    }

    normalize_likelihoods();
    double p_sum;
    double p;
    for (int i = 0; i < num_particles; ++i) {
        if (rand_prob(rng) > ratio) { //Spawn a certain amount of particles at observation
            if (rand_prob(rng) < p1) {
                next_node = edge->node1;
            } else {
                next_node = edge->node2;
            }
            particles[i] = std::make_shared<Particle>(*pose.get(), edge, next_node, graph, rand_speed(), rand_rot(), behavior, time);
            continue;
        }
        p_sum = 0;
        p = rand_prob(rng);
        for (int j = 0; j < num_particles; ++j) {
            p_sum += likelihoods[j];
            if (p_sum > p) {
                particles[i] = std::make_shared<Particle>(old_particles[j]->get_pose_at_time(time), old_particles[j]->get_edge_at_time(time), old_particles[j]->get_next_node_at_time(time), graph, rand_speed(), rand_rot(), behavior, time);
                break;
            }
        }
    }
}

void ParticleFilter::propogate_particles(double prop_time) { //TODO: diffuse likelihoods over time?
    for (auto& particle : particles) {
        particle->propogate(prop_time);
    }
}

double ParticleFilter::rand_speed() {
    return avg_speed + unit_normal(rng)*dev_speed;
}

double ParticleFilter::rand_rot() {
    return avg_rot + unit_normal(rng)*dev_rot;
}

void ParticleFilter::trim_history(double trim_time){
    for(auto& particle : particles){
        particle->trim_history(trim_time);
    }
}

double ParticleFilter::calculate_cost(std::shared_ptr<Trajectory> trajectory) {
    double total_cost = 0.0;
    double cost_inc = 1.0/num_particles;
    for(int i = 0; i < num_particles; i++){
        if(particles[i]->collision_check(trajectory)){
            total_cost += likelihoods[i];
        }
    }

    return total_cost;
}

double ParticleFilter::normal_pdf(double x, double mean, double stddev) {
    return (1.0 / (stddev * std::sqrt(2 * M_PI))) * std::exp(-0.5 * std::pow((x - mean) / stddev, 2));
}
