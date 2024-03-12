// TraversalEdge.hpp

#ifndef TRAVERSAL_EDGE_HPP
#define TRAVERSAL_EDGE_HPP

#include "TraversalNode.hpp"
#include <cmath>
#include <vector>

class TraversalEdge : public std::enable_shared_from_this<TraversalEdge> {
public:
    TraversalEdge(std::shared_ptr<TraversalNode> node1, std::shared_ptr<TraversalNode> node2);
    double get_distance_from(const std::vector<double>& position);
    void reset_color();
    std::shared_ptr<TraversalNode> get_connected_node(std::shared_ptr<TraversalNode> node);
    void link_nodes();
    
    std::shared_ptr<TraversalNode> node1;
    std::shared_ptr<TraversalNode> node2;

// private:
    std::vector<double> default_color;
    std::vector<double> draw_color;
    int draw_size;
    double base_cost;
    std::vector<std::vector<double>> cost_table;
    double dist;
};

#endif // TRAVERSAL_EDGE_HPP
