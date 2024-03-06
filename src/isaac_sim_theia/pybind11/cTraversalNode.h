#ifndef TRAVERSAL_NODE
#define TRAVERSAL_NODE

#include <vector>
#include <cmath>
#include <tuple>
#include <map>
#include <string>

#include "cTraversalEdge.h"

class TraversalNode {
public:
    std::vector<TraversalEdge*> edges;
    float x;
    float y;
    int key;
    float draw_height = 0.5;
    float default_color[4] = {1, 1, 1, 0.5};
    float draw_color[4] = {1, 1, 1, 0.5};
    int draw_size = 20;
    bool visited = false;

    TraversalNode(std::map<std::string, float> node, float draw_height = 0.5);
    std::vector<float> get_position();
    void reset_color();
    void add_edge(TraversalEdge* new_edge);
};

#endif