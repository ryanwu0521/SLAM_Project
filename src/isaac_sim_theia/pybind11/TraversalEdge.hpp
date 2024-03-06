// TraversalEdge.hpp

#ifndef TRAVERSAL_EDGE_HPP
#define TRAVERSAL_EDGE_HPP

#include "TraversalNode.hpp"
#include <cmath>
#include <vector>

class TraversalEdge {
public:
    TraversalEdge(TraversalNode* node1, TraversalNode* node2);
    double get_distance_from(const std::vector<double>& position);
    void reset_color();

    TraversalNode* node1;
    TraversalNode* node2;

private:
    std::vector<double> default_color;
    std::vector<double> draw_color;
    int draw_size;
    double base_cost;
    std::vector<std::vector<double>> cost_table;
    double dist;
};

#endif // TRAVERSAL_EDGE_HPP
