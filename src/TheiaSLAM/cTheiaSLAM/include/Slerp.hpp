#ifndef SLERP_HPP
#define SLERP_HPP

#include "Rotation.hpp"
#include <vector>

class Slerp
{
private:
    std::vector<Rotation> rotations_;
    std::vector<double> times_;

    // Method to perform SLERP interpolation between two rotations
    Rotation slerp(const Rotation& start, const Rotation& end, double interpolation_factor) const;

public:
    // Constructor
    Slerp(std::vector<Rotation> rots, std::vector<double> times);

    // Operator () to interpolate rotation at a given time
    Rotation operator()(double time) const;
};

#endif //SLERP_HPP
