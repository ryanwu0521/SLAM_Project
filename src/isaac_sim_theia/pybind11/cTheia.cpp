#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "include/TraversalGraph.hpp"

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

}