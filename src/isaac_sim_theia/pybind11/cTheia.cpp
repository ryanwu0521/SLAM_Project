#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "include/TraversalGraph.hpp"
#include "include/Slerp.hpp"
#include "include/Rotation.hpp"
#include "include/Pose.hpp"
#include "include/Trajectory.hpp"
#include "include/CompPath.hpp"
#include "include/Particle.hpp"
#include "include/ParticleFilter.hpp"

#include <pybind11/functional.h>
#include <pybind11/complex.h>

namespace py = pybind11;

//TODO: Need to trim back exposure to only what is necessary and make some things private

PYBIND11_MODULE(cTheia, m) {
    py::class_<TraversalEdge, std::shared_ptr<TraversalEdge>>(m, "TraversalEdge")
        .def(py::init<std::shared_ptr<TraversalNode>, std::shared_ptr<TraversalNode>>())
        .def("get_distance_from", &TraversalEdge::get_distance_from)
        .def("reset_color", &TraversalEdge::reset_color)
        .def("get_connected_node", &TraversalEdge::get_connected_node)
        .def_readwrite("node1",&TraversalEdge::node1)
        .def_readwrite("node2",&TraversalEdge::node2)
        .def_readwrite("default_color",&TraversalEdge::default_color)
        .def_readwrite("draw_color",&TraversalEdge::draw_color)
        .def_readwrite("draw_size",&TraversalEdge::draw_size)
        .def_readwrite("base_cost",&TraversalEdge::base_cost)
        .def_readwrite("dist",&TraversalEdge::dist);

    py::class_<TraversalNode, std::shared_ptr<TraversalNode>>(m, "TraversalNode")
        .def(py::init<const Json::Value&>())
        .def("get_position", &TraversalNode::get_position)
        .def("reset_color", &TraversalNode::reset_color)
        .def("add_edge", &TraversalNode::add_edge)
        .def_readwrite("x", &TraversalNode::x)
        .def_readwrite("y", &TraversalNode::y)
        .def_readwrite("key", &TraversalNode::key)
        .def_readwrite("edges", &TraversalNode::edges)
        .def_readwrite("draw_height", &TraversalNode::draw_height)
        .def_readwrite("default_color", &TraversalNode::default_color)
        .def_readwrite("draw_color", &TraversalNode::draw_color)
        .def_readwrite("draw_size", &TraversalNode::draw_size)
        .def_readwrite("visited", &TraversalNode::visited);

    py::class_<TraversalGraph, std::shared_ptr<TraversalGraph>>(m, "TraversalGraph")
        .def(py::init<std::string>())
        .def("load_file", &TraversalGraph::load_file)
        .def("add_edge", &TraversalGraph::add_edge)
        .def("validate_nodes", &TraversalGraph::validate_nodes)
        .def("get_closest_edge", &TraversalGraph::get_closest_edge)
        .def_readwrite("nodes",&TraversalGraph::nodes)
        .def_readwrite("edges",&TraversalGraph::edges); //TODO, possibly use setters/getters

    py::class_<Rotation, std::shared_ptr<Rotation>>(m, "Rotation")
        .def(py::init<std::array<double, 4>>())
        .def("as_quat", &Rotation::as_quat, py::arg("scalar_last") = true)
        .def("as_euler", (std::array<double, 3> (Rotation::*)(std::string, bool) const) &Rotation::as_euler, py::arg("order") = "XYZ", py::arg("degrees") = false)
        .def("as_euler", (std::array<double, 3> (Rotation::*)(const std::array<int, 3>&, bool) const) &Rotation::as_euler, py::arg("order"), py::arg("degrees") = false)
        .def("as_matrix", &Rotation::as_matrix)
        .def("__eq__", &Rotation::operator==)
        .def_static("from_euler", &Rotation::from_euler, py::arg("order"), py::arg("values"), py::arg("degrees") = false)
        .def_static("identity", &Rotation::identity)
        .def_static("from_quat", &Rotation::from_quat, py::arg("q"), py::arg("scalar_last") = true)
        .def_static("normalize_quaternion", &Rotation::normalize_quaternion)
        .def_static("multiply_quaternions", &Rotation::multiply_quaternions);

    py::class_<Slerp, std::shared_ptr<Slerp>>(m, "Slerp")
        .def(py::init<std::vector<Rotation>, std::vector<double>>())
        .def("__call__", &Slerp::operator());

    py::class_<Pose, std::shared_ptr<Pose>>(m, "Pose")
        .def(py::init<std::array<double, 3>, Rotation>(),
             py::arg("position") = std::array<double, 3>{0.0, 0.0, 0.0},
             py::arg("orientation") = Rotation::identity())
        .def("__eq__", &Pose::operator==)
        .def("set_heading_from_isaac_quat", &Pose::set_heading_from_isaac_quat)
        .def("get_quat_scalar_first", &Pose::get_quat_scalar_first)
        .def("get_quat_scalar_last", &Pose::get_quat_scalar_last)
        .def("set_heading_from_angle", &Pose::set_heading_from_angle, py::arg("angle"), py::arg("degrees") = true)
        .def("set_heading_to_destination", &Pose::set_heading_to_destination)
        .def("set_heading_from_origin", &Pose::set_heading_from_origin)
        .def("randomize_orientation", &Pose::randomize_orientation)
        .def("get_heading_from_orientation", &Pose::get_heading_from_orientation)
        .def("get_orientation", &Pose::get_orientation)
        .def("get_position", &Pose::get_position)
        .def("set_orientation", &Pose::set_orientation)
        .def("set_position", &Pose::set_position);

    py::class_<Trajectory, std::shared_ptr<Trajectory>>(m, "Trajectory")
        .def(py::init<Pose, Pose, double, double, double, double>(),
             py::arg("start_pose"), py::arg("end_pose"), py::arg("linear_speed"),
             py::arg("angular_speed"), py::arg("start_time"), py::arg("wait_time") = 0.001)
        .def(py::init<Pose, std::shared_ptr<TraversalNode>, std::shared_ptr<TraversalEdge>, double, double, double, double>(),
             py::arg("start_pose"), py::arg("next_node"), py::arg("edge"), py::arg("linear_speed"),
             py::arg("angular_speed"), py::arg("start_time"), py::arg("wait_time") = 0.001)
        .def("get_pose_at_time", &Trajectory::get_pose_at_time)
        .def("is_finished", &Trajectory::is_finished)
        .def_readonly("start_pose", &Trajectory::start_pose)
        .def_readonly("end_pose", &Trajectory::end_pose)
        .def_readonly("linear_speed", &Trajectory::linear_speed)
        .def_readonly("angular_speed", &Trajectory::angular_speed)
        .def_readonly("start_time", &Trajectory::start_time)
        .def_readonly("wait_time", &Trajectory::wait_time)
        .def_readonly("turn_time", &Trajectory::turn_time)
        .def_readonly("end_time", &Trajectory::end_time)
        .def_readonly("next_node", &Trajectory::next_node)
        .def_readonly("prev_node", &Trajectory::prev_node)
        .def_readonly("edge", &Trajectory::edge);

    py::class_<CompPath, std::shared_ptr<CompPath>>(m, "CompPath")
        .def(py::init<double, const std::vector<std::shared_ptr<TraversalNode>>&, double>(), py::arg("cost"), py::arg("path"), py::arg("time") = -1)
        .def("__eq__", &CompPath::operator==)
        .def("__ne__", &CompPath::operator!=)
        .def("__lt__", &CompPath::operator<)
        .def("__le__", &CompPath::operator<=)
        .def("__gt__", &CompPath::operator>)
        .def("__ge__", &CompPath::operator>=)
        .def_readonly("cost", &CompPath::cost)
        .def_readonly("path", &CompPath::path)
        .def_readonly("time", &CompPath::time);

    py::class_<Particle,std::shared_ptr<Particle>>(m, "Particle")
        .def(py::init<Pose, std::shared_ptr<TraversalEdge>, std::shared_ptr<TraversalNode>, std::shared_ptr<TraversalGraph>, double, double, const std::string&, double>())
        .def("collision_check", &Particle::collision_check)
        .def("add_next_traj", &Particle::add_next_traj)
        .def("trim_history", &Particle::trim_history)
        .def("get_pose_at_time", &Particle::get_pose_at_time)
        .def("get_edge_at_time", &Particle::get_edge_at_time)
        .def("get_next_node_at_time", &Particle::get_next_node_at_time)
        .def("_forward_behavior", &Particle::_forward_behavior)
        .def("_random_behavior", &Particle::_random_behavior)
        .def("propogate", &Particle::propogate)
        .def_readwrite("behavior", &Particle::behavior)
        .def_readwrite("linear_speed", &Particle::linear_speed)
        .def_readwrite("angular_speed", &Particle::angular_speed)
        .def_readwrite("graph", &Particle::graph)
        .def_readwrite("trajectory", &Particle::trajectory);

    py::class_<ParticleFilter, std::shared_ptr<ParticleFilter>>(m, "ParticleFilter")
        .def(py::init<std::unordered_map<std::string, std::string>, std::shared_ptr<TraversalGraph>, std::shared_ptr<Pose>, double>())
        .def("initialize_particles", &ParticleFilter::initialize_particles)
        .def("clear_visible_particles", &ParticleFilter::clear_visible_particles)
        .def("update_particles", &ParticleFilter::update_particles)
        .def("propogate_particles", &ParticleFilter::propogate_particles)
        .def("trim_history", &ParticleFilter::trim_history)
        .def("rand_speed", &ParticleFilter::rand_speed)
        .def("rand_rot", &ParticleFilter::rand_rot)
        .def("calculate_cost", &ParticleFilter::calculate_cost)
        .def("normalize_likelihoods", &ParticleFilter::normalize_likelihoods)
        .def_readwrite("last_update", &ParticleFilter::last_update)
        .def_readwrite("draw_size", &ParticleFilter::draw_size)
        .def_readwrite("draw_color", &ParticleFilter::draw_color)
        .def_readwrite("spawn_rate", &ParticleFilter::spawn_rate)
        .def_readwrite("avg_speed", &ParticleFilter::avg_speed)
        .def_readwrite("dev_speed", &ParticleFilter::dev_speed)
        .def_readwrite("avg_rot", &ParticleFilter::avg_rot)
        .def_readwrite("dev_rot", &ParticleFilter::dev_rot)
        .def_readwrite("behavior", &ParticleFilter::behavior)
        .def_readwrite("num_particles", &ParticleFilter::num_particles)
        .def_readwrite("position_variance", &ParticleFilter::position_variance)
        .def_readwrite("angle_variance", &ParticleFilter::angle_variance)
        .def_readwrite("update_rate", &ParticleFilter::update_rate);
}