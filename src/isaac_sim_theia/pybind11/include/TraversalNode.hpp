// TraversalNode.hpp

#ifndef TRAVERSAL_NODE_HPP
#define TRAVERSAL_NODE_HPP

#include "../jsoncpp/json/json.h" // Updated location for json/json.h
#include <vector>

class TraversalEdge; // Forward declaration

class TraversalNode : public std::enable_shared_from_this<TraversalNode> {
public:
    TraversalNode(const Json::Value& node);
    std::vector<double> get_position();
    void reset_color();
    void add_edge(std::shared_ptr<TraversalEdge> new_edge);

    double x;
    double y;
    int key;

// private:
    std::vector<std::shared_ptr<TraversalEdge>> edges;
    double draw_height;
    std::vector<double> default_color;
    std::vector<double> draw_color;
    int draw_size;
    bool visited;
};

#endif // TRAVERSAL_NODE_HPP
