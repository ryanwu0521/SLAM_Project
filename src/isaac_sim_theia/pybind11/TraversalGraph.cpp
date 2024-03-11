// TraversalGraph.cpp

#include "../jsoncpp/json/json.h"
#include "include/TraversalGraph.hpp"
#include "include/TraversalNode.hpp"
#include "include/TraversalEdge.hpp"

#include <iostream>
#include <fstream>


TraversalGraph::TraversalGraph(std::string file_string) {
    load_file(file_string);
}

void TraversalGraph::load_file(std::string file_string) {
    std::cout << "Loading graph from file " << file_string << std::endl;
    
    // Open the JSON file
    std::ifstream file(file_string);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open JSON file");
    }

    // Parse the JSON data
    Json::CharReaderBuilder builder;
    Json::Value data;
    std::string errors;
    if (!Json::parseFromStream(builder, file, &data, &errors)) {
        throw std::runtime_error("Failed to parse JSON: " + errors);
    }

    // Load nodes from JSON data
    const Json::Value& nodesJson = data["nodes"];
    for (const auto& nodeJson : nodesJson) {
        nodes.push_back(new TraversalNode(nodeJson));
    }

    // Validate nodes
    validate_nodes();

    // Load edges from JSON data
    const Json::Value& edgesJson = data["edges"];
    for (const auto& edgeJson : edgesJson) {
        add_edge(edgeJson);
    }

    std::cout << "Traversal graph loaded successfully" << std::endl;
}

void TraversalGraph::add_edge(const Json::Value& edge) {
    int node1_key = edge["node1"].asInt();
    int node2_key = edge["node2"].asInt();

    if (node1_key == node2_key) {
        throw std::runtime_error("Self-referential edge in JSON file");
    } else if (node1_key >= nodes.size() || node2_key >= nodes.size()) {
        throw std::runtime_error("Invalid key for node");
    }

    TraversalNode *fnode1 = nodes[node1_key];
    TraversalNode *fnode2 = nodes[node2_key];

    edges.emplace_back(new TraversalEdge(fnode1, fnode2));
}

void TraversalGraph::validate_nodes() {
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (i != nodes[i]->key) {
            throw std::runtime_error("Node key mismatch, check input JSON file structure");
        }
    }
}

TraversalEdge *TraversalGraph::get_closest_edge(std::vector<double>& position) {
    double closest = std::numeric_limits<double>::infinity();
    TraversalEdge *cedge = nullptr;

    for (auto& edge : edges) {
        double d2e = edge->get_distance_from(position);
        if (d2e == 0) {
            return edge;
        }
        if (d2e < closest) {
            closest = d2e;
            cedge = edge;
        }
    }
    return cedge;
}


