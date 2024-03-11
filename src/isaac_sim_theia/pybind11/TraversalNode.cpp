// TraversalNode.cpp

#include "include/TraversalNode.hpp"
#include "include/TraversalEdge.hpp"
TraversalNode::TraversalNode(const Json::Value& node) {
    x = node["x"].asDouble();
    y = node["y"].asDouble();
    key = node["key"].asInt();
    draw_height = 0.5;
    default_color = {1.0, 1.0, 1.0, 0.5};
    draw_color = default_color;
    draw_size = 20;
    visited = false;
}

std::vector<double> TraversalNode::get_position() {
    return {x, y, draw_height};
}

void TraversalNode::reset_color() {
    draw_color = default_color;
}

void TraversalNode::add_edge(TraversalEdge* new_edge) {
    for (auto edge : edges) {
        if (new_edge->node1 == this && (edge->node1 == new_edge->node2 || edge->node2 == new_edge->node2)) {
            throw std::runtime_error("Attempted to add duplicate edge");
        } else if (new_edge->node2 == this && (edge->node1 == new_edge->node1 || edge->node2 == new_edge->node1)) {
            throw std::runtime_error("Attempted to add duplicate edge");
        }
    }
    edges.push_back(new_edge);
}
