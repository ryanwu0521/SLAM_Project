#include <vector>
#include <cmath>

#include "cTraversalEdge.h"


TraversalNode::TraversalNode(std::map<std::string, float> node, float dheight = 0.5) {
    x = node["x"];
    y = node["y"];
    key = node["key"];

    draw_height = dheight;
}

std::vector<float> TraversalNode::get_position() {
    return {x, y, draw_height};
}

void TraversalNode::reset_color() {
    draw_color = default_color;
}

void TraversalNode::add_edge(TraversalEdge* new_edge) {
    if (new_edge->node1 == this) {
        for (auto edge : edges) {
            if (new_edge->node2 == edge->node1 || new_edge->node2 == edge->node2) {
                throw std::exception("Attempted to add duplicate edge");
            }
        }
    }
    else if (new_edge->node2 == this) {
        for (auto edge : edges) {
            if (new_edge->node1 == edge->node1 || new_edge->node1 == edge->node2) {
                throw std::exception("Attempted to add duplicate edge");
            }
        }
    }
    else {
        throw std::exception("Attempted to add edge to non-member node");
    }
    edges.push_back(new_edge);
}
