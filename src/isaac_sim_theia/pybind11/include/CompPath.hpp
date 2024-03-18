#ifndef COMP_PATH_HPP
#define COMP_PATH_HPP

#include <memory>
#include <vector>

class TraversalNode;

class CompPath {
public:
    CompPath(double cost, const std::vector<std::shared_ptr<TraversalNode>> path, double time = -1);
    
    bool operator==(const CompPath& other) const;
    bool operator!=(const CompPath& other) const;
    bool operator<(const CompPath& other) const;
    bool operator<=(const CompPath& other) const;
    bool operator>(const CompPath& other) const;
    bool operator>=(const CompPath& other) const;
    double cost;
    std::vector<std::shared_ptr<TraversalNode>> path;
    double time;
};

#endif // COMP_PATH_HPP
