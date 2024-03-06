// TraversalEdge.cpp

#include "TraversalEdge.hpp"

TraversalEdge::TraversalEdge(TraversalNode* node1, TraversalNode* node2) {
    this->node1 = node1;
    this->node2 = node2;
    default_color = {1.0, 0.0, 0.0, 0.25};
    draw_color = default_color;
    draw_size = 5;
    base_cost = 0;
    dist = sqrt(pow(node1->x - node2->x, 2) + pow(node1->y - node2->y, 2));
}

double TraversalEdge::get_distance_from(const std::vector<double>& position) {
    std::vector<double> A = {node1->x, node1->y};
    std::vector<double> B = {node2->x, node2->y};
    std::vector<double> E = {position[0], position[1]};

    std::vector<double> AB = {B[0] - A[0], B[1] - A[1]};
    std::vector<double> AE = {E[0] - A[0], E[1] - A[1]};
    std::vector<double> BE = {E[0] - B[0], E[1] - B[1]};

    double AB_BE = AB[0] * BE[0] + AB[1] * BE[1];
    double AB_AE = AB[0] * AE[0] + AB[1] * AE[1];

    if (AB_BE > 0) {
        return sqrt(pow(E[0] - B[0], 2) + pow(E[1] - B[1], 2));
    } else if (AB_AE < 0) {
        return sqrt(pow(E[0] - A[0], 2) + pow(E[1] - A[1], 2));
    } else {
        return abs(AB[0] * AE[1] - AB[1] * AE[0]) / sqrt(pow(AB[0], 2) + pow(AB[1], 2));
    }
}

void TraversalEdge::reset_color() {
    draw_color = default_color;
}
