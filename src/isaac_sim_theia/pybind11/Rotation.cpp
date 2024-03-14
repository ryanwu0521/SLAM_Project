#include "include/Rotation.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm> // for std::transform
#include <cctype>    // for std::toupper
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Rotation::Rotation(std::array<double, 4> q) : _q(q) {}

Rotation::~Rotation() {}

// Method to retrieve the quaternion
std::array<double, 4> Rotation::as_quat() const{
    return _q;
}

std::array<double, 3> Rotation::as_euler(std::string order, bool degrees) const{
// Method to convert to Euler anglesstd::array<double, 3> Rotation::as_euler(std::string order, bool degrees) const{
    // Extract quaternion components
    double qw = _q[3];
    double qx = _q[0];
    double qy = _q[1];
    double qz = _q[2];

    std::transform(order.begin(), order.end(), order.begin(), [](unsigned char c) { return std::toupper(c); });

    // Compute Euler angles based on the specified order
    std::array<double, 3> euler;

    if (order == "XYZ") {
        // Handle XYZ order
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp); // Roll (X-axis rotation)

        double sinp = 2 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            euler[1] = std::copysign(M_PI / 2, sinp); // Pitch (Y-axis rotation)
        else
            euler[1] = std::asin(sinp);

        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        euler[2] = std::atan2(siny_cosp, cosy_cosp); // Yaw (Z-axis rotation)
    } else if (order == "XZY") {
        // Handle XZY order
        double sinr_cosp = 2 * (qw * qx - qz * qy);
        double cosr_cosp = 1 - 2 * (qx * qx + qz * qz);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp); // Roll (X-axis rotation)

        double sinp = 2 * (qw * qz + qx * qy);
        if (std::abs(sinp) >= 1)
            euler[1] = std::copysign(M_PI / 2, sinp); // Pitch (Z-axis rotation)
        else
            euler[1] = std::asin(sinp);

        double siny_cosp = 2 * (qw * qy + qz * qx);
        double cosy_cosp = 1 - 2 * (qx * qx + qy * qy);
        euler[2] = std::atan2(siny_cosp, cosy_cosp); // Yaw (Y-axis rotation)
    } else if (order == "YXZ") {
        // Handle YXZ order
        double sinr_cosp = 2 * (qw * qy - qz * qx);
        double cosr_cosp = 1 - 2 * (qx * qx + qz * qz);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp); // Roll (Y-axis rotation)

        double sinp = 2 * (qw * qx + qy * qz);
        if (std::abs(sinp) >= 1)
            euler[1] = std::copysign(M_PI / 2, sinp); // Pitch (X-axis rotation)
        else
            euler[1] = std::asin(sinp);

        double siny_cosp = 2 * (qw * qz + qy * qx);
        double cosy_cosp = 1 - 2 * (qx * qx + qy * qy);
        euler[2] = std::atan2(siny_cosp, cosy_cosp); // Yaw (Z-axis rotation)
    } else if (order == "YZX") {
        // Handle YZX order
        double sinr_cosp = 2 * (qw * qy + qz * qx);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp); // Roll (Y-axis rotation)

        double sinp = 2 * (qw * qx - qy * qz);
        if (std::abs(sinp) >= 1)
            euler[1] = std::copysign(M_PI / 2, sinp); // Pitch (X-axis rotation)
        else
            euler[1] = std::asin(sinp);

        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        euler[2] = std::atan2(siny_cosp, cosy_cosp); // Yaw (Z-axis rotation)
    } else if (order == "ZXY") {
        // Handle ZXY order
        double sinr_cosp = 2 * (qw * qz - qx * qy);
        double cosr_cosp = 1 - 2 * (qy * qy + qz * qz);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp); // Roll (Z-axis rotation)

        double sinp = 2 * (qw * qx + qy * qz);
        if (std::abs(sinp) >= 1)
            euler[1] = std::copysign(M_PI / 2, sinp); // Pitch (X-axis rotation)
        else
            euler[1] = std::asin(sinp);

        double siny_cosp = 2 * (qw * qy + qz * qx);
        double cosy_cosp = 1 - 2 * (qx * qx + qz * qz);
        euler[2] = std::atan2(siny_cosp, cosy_cosp); // Yaw (Y-axis rotation)
    } else if (order == "ZYX") {
        // Handle ZYX order
        double sinr_cosp = 2 * (qw * qz + qx * qy);
        double cosr_cosp = 1 - 2 * (qy * qy + qz * qz);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp); // Roll (Z-axis rotation)

        double sinp = 2 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            euler[1] = std::copysign(M_PI / 2, sinp); // Pitch (Y-axis rotation)
        else
            euler[1] = std::asin(sinp);

        double siny_cosp = 2 * (qw * qx + qy * qz);
        double cosy_cosp = 1 - 2 * (qx * qx + qy * qy);
        euler[2] = std::atan2(siny_cosp, cosy_cosp); // Yaw (X-axis rotation)
    } else {
        throw std::invalid_argument("Invalid Euler angle order");
    }

    // Convert to degrees if required
    if (degrees) {
        for (auto& angle : euler) {
            angle = angle * 180.0 / M_PI;
        }
    }

    return euler;
}


// Static method to create a Rotation object from Euler angles
Rotation Rotation::from_euler(std::string order, std::vector<double> values, bool degrees) {
    // Convert angles to radians if required
    if (degrees) {
        for (auto& angle : values) {
            angle = angle * M_PI / 180.0;
        }
    }

    // Convert order string to lowercase
    std::transform(order.begin(), order.end(), order.begin(), [](unsigned char c) { return std::toupper(c); });

    // Extract Euler angles based on the specified order
    double angles[3] = {0.0, 0.0, 0.0};
    if (order == "XYZ" || order == "X") {
        angles[0] = values[0];   // Roll
        angles[1] = values[1];   // Pitch
        angles[2] = values[2];   // Yaw
    } else if (order == "XZY") {
        angles[0] = values[0];   // Roll
        angles[1] = values[2];   // Yaw
        angles[2] = values[1];   // Pitch
    } else if (order == "YXZ" || order == "Y") {
        angles[0] = values[1];   // Pitch
        angles[1] = values[0];   // Roll
        angles[2] = values[2];   // Yaw
    } else if (order == "YZX") {
        angles[0] = values[1];   // Pitch
        angles[1] = values[2];   // Yaw
        angles[2] = values[0];   // Roll
    } else if (order == "ZXY" || order == "Z") {
        angles[0] = values[2];   // Yaw
        angles[1] = values[0];   // Roll
        angles[2] = values[1];   // Pitch
    } else if (order == "ZYX") {
        angles[0] = values[2];   // Yaw
        angles[1] = values[1];   // Pitch
        angles[2] = values[0];   // Roll
    } else {
        throw std::invalid_argument("Invalid Euler angle order");
    }

    std::cout << angles[0] << ", " << angles[1] << ", " << angles[2] << std::endl;

    // Compute quaternion components based on the specified order
    double cy = std::cos(angles[2] * 0.5);
    double sy = std::sin(angles[2] * 0.5);
    double cp = std::cos(angles[1] * 0.5);
    double sp = std::sin(angles[1] * 0.5);
    double cr = std::cos(angles[0] * 0.5);
    double sr = std::sin(angles[0] * 0.5);

    // Check for NaN values
    if (std::isnan(cy) || std::isnan(sy) || std::isnan(cp) || std::isnan(sp) || std::isnan(cr) || std::isnan(sr)) {
        throw std::runtime_error("Computed NaN values while converting Euler angles to quaternion");
    }

    std::array<double, 4> q1 = {sr, 0.0, 0.0, cr}; // Quaternion for roll
    std::array<double, 4> q2 = {0.0, sp, 0.0, cp}; // Quaternion for pitch
    std::array<double, 4> q3 = {0.0, 0.0, sy, cy}; // Quaternion for yaw

    // Combine the three rotations using quaternion multiplication
    std::array<double, 4> combined_quaternion = multiply_quaternions(q1, multiply_quaternions(q2, q3));

    // Normalize the resulting quaternion to ensure it represents a valid rotation
    combined_quaternion = normalize_quaternion(combined_quaternion);

    return Rotation(combined_quaternion);
}



// Helper function to normalize a quaternion
std::array<double, 4> Rotation::normalize_quaternion(const std::array<double, 4>& q) {
    double norm_squared = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (norm_squared == 0.0) {
        throw std::runtime_error("Attempted to normalize a zero quaternion");
    }
    double norm = std::sqrt(norm_squared);
    return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}



// Static method to create a Rotation object from a quaternion
Rotation Rotation::from_quat(std::array<double, 4> q) {
    return Rotation(q);
}

// Helper function to multiply two quaternions
std::array<double, 4> Rotation::multiply_quaternions(const std::array<double, 4>& q1, const std::array<double, 4>& q2) {
    return {
        q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3], // w
        q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2], // x
        q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1], // y
        q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]  // z
    };
}

Rotation Rotation::identity(){
    return Rotation();
}
