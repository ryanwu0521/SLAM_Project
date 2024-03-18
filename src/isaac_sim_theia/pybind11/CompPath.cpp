#include "include/CompPath.hpp"
#include "include/TraversalNode.hpp" // You should replace this with the appropriate header file for TraversalNode

CompPath::CompPath(double cost, const std::vector<std::shared_ptr<TraversalNode>> path, double time) 
    : cost(cost), path(path), time((time == -1) ? cost : time) {}

bool CompPath::operator==(const CompPath& other) const {
    return cost == other.cost;
}

bool CompPath::operator!=(const CompPath& other) const {
    return cost != other.cost;
}

bool CompPath::operator<(const CompPath& other) const {
    return cost < other.cost;
}

bool CompPath::operator<=(const CompPath& other) const {
    return cost <= other.cost;
}

bool CompPath::operator>(const CompPath& other) const {
    return cost > other.cost;
}

bool CompPath::operator>=(const CompPath& other) const {
    return cost >= other.cost;
}