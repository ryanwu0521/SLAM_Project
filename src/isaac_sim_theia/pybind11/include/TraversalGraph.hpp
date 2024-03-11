// TraversalGraph.hpp

#ifndef TRAVERSAL_GRAPH_HPP
#define TRAVERSAL_GRAPH_HPP

#include "TraversalNode.hpp"
#include "TraversalEdge.hpp"
#include "../jsoncpp/json/json.h" // Updated location for json/json.h
#include <vector>
#include <string>

class TraversalGraph {
public:
    TraversalGraph(std::string file_string);
    void load_file(std::string file_string);
    void add_edge(const Json::Value& edge);
    void validate_nodes();
    TraversalEdge *get_closest_edge(std::vector<double>& position);

// private:
    std::vector<TraversalNode *> nodes;
    std::vector<TraversalEdge *> edges;
};

#endif // TRAVERSAL_GRAPH_HPP
