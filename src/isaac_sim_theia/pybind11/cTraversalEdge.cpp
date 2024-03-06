#include "cTraversalNode.h"
#include <vector>
#include <cmath>
 
TraversalEdge::TraversalEdge(TraversalNode* n1, TraversalNode* n2) {
    node1 = n1;
    node2 = n2;

    dist = sqrt(pow(node1->x - node2->x, 2) + pow(node1->y - node2->y, 2));

    node1->add_edge(this);
    node2->add_edge(this);
}

TraversalNode* TraversalEdge::get_connected_node(TraversalNode* node) {
    if (node->key == node1->key) {
        return node2;
    }
    if (node->key == node2->key) {
        return node1;
    }
    throw std::exception("Node not part of edge");
}

float TraversalEdge::get_distance_from(float position[3]) {
    float A[2] = {node1->x, node1->y};
    float B[2] = {node2->x, node2->y};
    float E[2] = {position[0], position[1]};

    float AB[2] = {B[0] - A[0], B[1] - A[1]};
    float AE[2] = {E[0] - A[0], E[1] - A[1]};
    float BE[2] = {E[0] - B[0], E[1] - B[1]};

    float AB_BE = AB[0] * BE[0] + AB[1] * BE[1];
    float AB_AE = AB[0] * AE[0] + AB[1] * AE[1];

    float reqAns = 0;

    if (AB_BE > 0) {
        reqAns = sqrt(pow(E[0] - B[0], 2) + pow(E[1] - B[1], 2));
    }
    else if (AB_AE < 0) {
        reqAns = sqrt(pow(E[0] - A[0], 2) + pow(E[1] - A[1], 2));
    }
    else {
        reqAns = abs(AB[0] * AE[1] - AB[1] * AE[0]) / sqrt(pow(AB[0], 2) + pow(AB[1], 2));
    }
    return reqAns;
}

void TraversalEdge::reset_color() {
    for(int i =0; i < 4; i++){
        draw_color[i] = default_color[i];
    }
}