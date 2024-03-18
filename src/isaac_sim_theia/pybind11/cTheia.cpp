#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "include/TraversalGraph.hpp"
#include "include/Slerp.hpp"
#include "include/Rotation.hpp"
#include "include/Pose.hpp"
#include "include/Trajectory.hpp"
#include "include/CompPath.hpp"

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

    py::class_<TraversalGraph>(m, "TraversalGraph")
        .def(py::init<std::string>())
        .def("load_file", &TraversalGraph::load_file)
        .def("add_edge", &TraversalGraph::add_edge)
        .def("validate_nodes", &TraversalGraph::validate_nodes)
        .def("get_closest_edge", &TraversalGraph::get_closest_edge)
        .def_readwrite("nodes",&TraversalGraph::nodes)
        .def_readwrite("edges",&TraversalGraph::edges); //TODO, possibly use setters/getters

    py::class_<Rotation>(m, "Rotation")
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

    py::class_<Slerp>(m, "Slerp")
        .def(py::init<std::vector<Rotation>, std::vector<double>>())
        .def("__call__", &Slerp::operator());

    py::class_<Pose>(m, "Pose")
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

    py::class_<Trajectory>(m, "Trajectory")
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

    py::class_<CompPath>(m, "CompPath")
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
}