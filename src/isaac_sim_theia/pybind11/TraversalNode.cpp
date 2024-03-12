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

void TraversalNode::add_edge(std::shared_ptr<TraversalEdge> new_edge) {
    for (auto edge : edges) {
        if (new_edge->node1.get() == this && (edge->node1.get() == new_edge->node2.get() || edge->node2.get() == new_edge->node2.get())) {
            throw std::runtime_error("Attempted to add duplicate edge");
        } else if (new_edge->node2.get() == this && (edge->node1.get() == new_edge->node1.get() || edge->node2.get() == new_edge->node1.get())) {
            throw std::runtime_error("Attempted to add duplicate edge");
        }
    }
    edges.push_back(new_edge);
}
