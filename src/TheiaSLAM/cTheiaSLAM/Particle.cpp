#include "include/Particle.hpp"
#include <random>
// #include <optional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Particle::Particle(Pose pose, const std::shared_ptr<TraversalEdge> edge, const std::shared_ptr<TraversalNode> next_node, const std::shared_ptr<TraversalGraph> fgraph, double speed, double rot, const std::string behavior, double time)
    : behavior(behavior), linear_speed(speed), angular_speed(rot), graph(fgraph) {
    trajectory.emplace_back(std::make_shared<Trajectory>(pose, next_node, edge, linear_speed, angular_speed, time));
}

bool Particle::collision_check(const std::shared_ptr<Trajectory> check_trajectory) {
    propogate(check_trajectory->end_time);
    for (const std::shared_ptr<Trajectory> traj_piece : trajectory) {
        if (traj_piece->start_time > check_trajectory->end_time + 1)
            continue;
        else if (traj_piece->end_time < check_trajectory->start_time - 1)
            continue;

        bool A = (traj_piece->prev_node->key != check_trajectory->prev_node->key);
        bool B = (traj_piece->prev_node->key != check_trajectory->next_node->key);
        bool C = (traj_piece->next_node->key != check_trajectory->next_node->key);
        bool D = (traj_piece->next_node->key != check_trajectory->prev_node->key);

        if (A && B && C && D)
            continue;

        if (!B && !C)
            return true;

        if (!A) {
            if (traj_piece->turn_time < check_trajectory->start_time - 1)
                return true;
            if (check_trajectory->turn_time < traj_piece->start_time - 1)
                return true;
        }

        if (!B && traj_piece->turn_time < check_trajectory->end_time - 1)
            return true;

        if (!C && std::abs(traj_piece->end_time - check_trajectory->end_time) < 2)
            return true;

        if (!D && check_trajectory->turn_time < traj_piece->end_time - 1)
            return true;
    }
    return false;
}

void Particle::add_next_traj() {
    if (behavior == "random")
        _random_behavior();
    else if (behavior == "forward")
        _forward_behavior();
    else
        _random_behavior();
}

void Particle::_forward_behavior() {
    std::shared_ptr<Trajectory> end_traj = trajectory.back();
    std::shared_ptr<TraversalNode> end_node = end_traj->next_node;

    double heading = end_traj->end_pose.get_heading_from_orientation();
    std::random_device rd;
    std::default_random_engine gen(rd());
    std::normal_distribution<double> heading_distribution(heading, 50); // TODO: Parameterize
    double rand_heading = heading_distribution(gen);

    while (rand_heading < -180)
        rand_heading += 360;
    while (rand_heading > 180)
        rand_heading -= 360;

    double smallest_turn = std::numeric_limits<double>::infinity();
    std::shared_ptr<TraversalEdge> best_edge;
    std::shared_ptr<TraversalNode> next_node;

    for (std::shared_ptr<TraversalEdge> edge : end_node->edges) {
        auto test_node = edge->get_connected_node(end_node);
        double edge_heading = std::atan2(test_node->y - end_node->y, test_node->x - end_node->x) * 180.0 / M_PI;
        double dturn = std::abs(edge_heading - rand_heading);
        if (dturn > 180)
            dturn = 360 - dturn;
        if (dturn < smallest_turn) {
            smallest_turn = dturn;
            next_node = test_node;
            best_edge = edge;
        }
    }

    Pose start_pose = end_traj->end_pose;
    double time = end_traj->end_time;
    trajectory.emplace_back(std::make_shared<Trajectory>(start_pose, next_node, best_edge, linear_speed, angular_speed, time));
}

void Particle::_random_behavior() {
    std::shared_ptr<Trajectory> end_traj = trajectory.back();
    std::shared_ptr<TraversalNode> end_node = end_traj->next_node;

    std::random_device rd;
    std::default_random_engine gen(rd());
    std::uniform_int_distribution<size_t> distribution(0, end_node->edges.size() - 1);
    size_t index = distribution(gen);
    std::shared_ptr<TraversalEdge> edge = end_node->edges[index];
    std::shared_ptr<TraversalNode> next_node = edge->get_connected_node(end_node);

    Pose start_pose = end_traj->end_pose;
    double time = end_traj->end_time;
    trajectory.emplace_back(std::make_shared<Trajectory>(start_pose, next_node, edge, linear_speed, angular_speed, time));
}

Pose Particle::get_pose_at_time(double time) {
    if (trajectory.empty())
        throw std::runtime_error("trajectory empty for some reason");
    else if (time < trajectory.front()->start_time)
        throw std::runtime_error("tried to pull a particle history from too far into the past");

    if (time > trajectory.back()->end_time)
        propogate(time);

    for (const std::shared_ptr<Trajectory> traj_piece : trajectory) {
        if (time >= traj_piece->start_time && time <= traj_piece->end_time)
            return traj_piece->get_pose_at_time(time);
    }

    throw std::runtime_error("This should never happen");
}

std::shared_ptr<TraversalEdge> Particle::get_edge_at_time(double time){
    if (trajectory.empty())
        throw std::runtime_error("trajectory empty for some reason");
    else if (time < trajectory.front()->start_time)
        throw std::runtime_error("tried to pull a particle history from too far into the past");

    if (time > trajectory.back()->end_time)
        propogate(time);

    for (const std::shared_ptr<Trajectory> traj_piece : trajectory) {
        if (time >= traj_piece->start_time && time <= traj_piece->end_time)
            return traj_piece->edge;
    }

    throw std::runtime_error("This should never happen");
}

std::shared_ptr<TraversalNode> Particle::get_next_node_at_time(double time){
    if (trajectory.empty())
        throw std::runtime_error("trajectory empty for some reason");
    else if (time < trajectory.front()->start_time)
        throw std::runtime_error("tried to pull a particle history from too far into the past");

    if (time > trajectory.back()->end_time)
        propogate(time);

    for (const std::shared_ptr<Trajectory> traj_piece : trajectory) {
        if (time >= traj_piece->start_time && time <= traj_piece->end_time)
            return traj_piece->next_node;
    }

    throw std::runtime_error("This should never happen");
}

void Particle::propogate(double time) {
    if (trajectory.empty())
        throw std::runtime_error("somehow trajectory is empty");

    while (trajectory.back()->end_time < time)
        add_next_traj();
}

void Particle::trim_history(double time) {
    if (trajectory.empty()) {
        // warnings.warn("can't trim empty history");
        return;
    }

    while (trajectory.front()->end_time < time)
        trajectory.erase(trajectory.begin());
}
