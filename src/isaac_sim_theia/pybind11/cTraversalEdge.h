#ifndef TRAVERSAL_EDGE
#define TRAVERSAL_EDGE

#include <vector>
#include <cmath>
#include <tuple>
#include "cTraversalNode.h"

class TraversalEdge {
public:
    TraversalNode* node1;
    TraversalNode* node2;
    float default_color[4] = {1, 0, 0, 0.25};
    float draw_color[4] = {1, 0, 0, 0.25} ;

    int draw_size = 5;
    float base_cost = 0;
    float dist;

    TraversalEdge(TraversalNode* n1, TraversalNode* n2);
    TraversalNode* get_connected_node(TraversalNode* node);
    float get_distance_from(float position[3]);
    void reset_color();
};

#endif