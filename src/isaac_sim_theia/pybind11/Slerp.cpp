#include "include/Slerp.hpp"
#include <cmath>
#include <stdexcept>

// Constructor
Slerp::Slerp(std::vector<Rotation> rots, std::vector<double> times)
    : rotations_(rots), times_(times) {}

// Method to perform SLERP interpolation between two rotations
Rotation Slerp::slerp(const Rotation& start, const Rotation& end, double interpolation_factor) const {
    // Get quaternions from Rotation objects
    std::array<double, 4> q1 = start.as_quat();
    std::array<double, 4> q2 = end.as_quat();

    // Dot product of the quaternions
    double dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

    // Ensure the dot product is within bounds
    if (dot < 0.0) {
        // Reverse the sign of one quaternion to take the shorter path
        q2 = {-q2[0], -q2[1], -q2[2], -q2[3]};
        dot = -dot;
    }

    // Clamp dot product to avoid numerical instability
    dot = std::max(-1.0, std::min(1.0, dot));

    // Calculate the angle between the two quaternions
    double theta_0 = std::acos(dot);
    double theta = theta_0 * interpolation_factor;

    // Compute the coefficients for linear interpolation
    double sin_theta = std::sin(theta);
    double sin_theta_0 = std::sin(theta_0);
    double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
    double s1 = sin_theta / sin_theta_0;

    // Perform SLERP interpolation
    std::array<double, 4> result;
    for (size_t i = 0; i < 4; ++i) {
        result[i] = s0 * q1[i] + s1 * q2[i];
    }

    // Return the interpolated quaternion as a Rotation object
    return Rotation(result);
}



// Operator () to interpolate rotation at a given time
Rotation Slerp::operator()(double time) const {
    // Check if the times vector is empty
    if (times_.empty()) {
        throw std::runtime_error("Times vector is empty");
    }

    // Check if the provided time is out of bounds
    if (time < times_.front() || time > times_.back()) {
        throw std::invalid_argument("Time is out of bounds");
    }

    // Find the two rotations between which to interpolate
    size_t index = 0;
    for (size_t i = 0; i < times_.size(); ++i) {
        if (time <= times_[i]) {
            index = i;
            break;
        }
    }

    // Interpolation factor
    double factor = (time - times_[index]) / (times_[index + 1] - times_[index]);

    // Perform SLERP interpolation
    return slerp(rotations_[index], rotations_[index + 1], factor);
}
