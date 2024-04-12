// TraversalGraph.cpp

#include "../external/jsoncpp/json/json.h"
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
    nodes.reserve(nodesJson.size());
    for (const auto& nodeJson : nodesJson) {
        std::shared_ptr<TraversalNode> addNode = std::make_shared<TraversalNode>(nodeJson);
        nodes.push_back(addNode);
    }

    // Validate nodes
    validate_nodes();

    // Load edges from JSON data
    const Json::Value& edgesJson = data["edges"];
    edges.reserve(edgesJson.size());
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
    std::shared_ptr<TraversalEdge> temp = std::make_shared<TraversalEdge>(nodes[node1_key], nodes[node2_key]);
    temp->link_nodes();
    edges.emplace_back(std::make_shared<TraversalEdge>(nodes[node1_key], nodes[node2_key]));
}

void TraversalGraph::validate_nodes() {
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (i != nodes[i]->key) {
            throw std::runtime_error("Node key mismatch, check input JSON file structure");
        }
    }
}

std::shared_ptr<TraversalEdge> TraversalGraph::get_closest_edge(std::array<double, 3>& position) {
    double closest = std::numeric_limits<double>::infinity();
    std::shared_ptr<TraversalEdge> cedge = nullptr;

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


