#include "include/Rotation.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm> // for std::transform
#include <cctype>    // for std::toupper
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923 /* pi/2 */
#endif


Rotation::Rotation(std::array<double, 4> q) : _q(q) {
    normalize_quaternion(_q);
}

Rotation::~Rotation() {}

// Method to retrieve the quaternion
std::array<double, 4> Rotation::as_quat(bool scalar_last) const{
    if(scalar_last == false){
        return {_q[3], _q[0], _q[1], _q[2]};
    }
    return _q;
}

std::array<std::array<double, 3>, 3> Rotation::as_matrix() const {
    double qw = _q[3];
    double qx = _q[0];
    double qy = _q[1];
    double qz = _q[2];

    std::array<std::array<double, 3>, 3> R;

    R[0][0] = 1 - 2 * (qy * qy + qz * qz);
    R[0][1] = 2 * (qx * qy - qz * qw);
    R[0][2] = 2 * (qx * qz + qy * qw);

    R[1][0] = 2 * (qx * qy + qz * qw);
    R[1][1] = 1 - 2 * (qx * qx + qz * qz);
    R[1][2] = 2 * (qy * qz - qx * qw);

    R[2][0] = 2 * (qx * qz - qy * qw);
    R[2][1] = 2 * (qy * qz + qx * qw);
    R[2][2] = 1 - 2 * (qx * qx + qy * qy);

    return R;
}


std::array<double, 3> Rotation::as_euler(const std::array<int, 3>& order, bool degrees) const{
    std::array<double, 3> euler;

    // Extract Euler angles from the rotation matrix based on the order
    std::array<std::array<double, 3>, 3> R = as_matrix();

    double sp = -R[order[2]][order[0]]; // sin(theta)
    double cp = std::sqrt(1 - sp * sp);  // cos(theta)

    // Handle near-gimbal lock situations
    if (std::abs(cp) < 1e-6) {
        // Gimbal lock: theta is near +/- pi/2
        euler[1] = sp > 0 ? M_PI_2 : -M_PI_2;
        euler[0] = std::atan2(R[order[0]][order[1]], R[order[1]][order[1]]);
        euler[2] = 0; // Set third angle to 0 for consistency
    } else {
        euler[1] = std::asin(sp);
        euler[0] = std::atan2(R[order[2]][order[1]] / cp, R[order[2]][order[2]] / cp);
        euler[2] = std::atan2(R[order[1]][order[0]] / cp, R[order[0]][order[0]] / cp);
    }

    // Convert to degrees if required
    if (degrees) {
        for (auto& angle : euler) {
            angle = angle * 180.0 / M_PI;
        }
    }

    return euler;
}


// Method to convert to Euler angles
std::array<double, 3> Rotation::as_euler(std::string order, bool degrees) const{

    std::transform(order.begin(), order.end(), order.begin(), [](unsigned char c) { return std::toupper(c); });

    std::array<int, 3> index_order;

    if (order == "XYZ") {
        index_order = {0,1,2};
    } else if (order == "XZY") {
        index_order = {0,2,1};
    } else if (order == "YXZ") {
        index_order = {1,0,2};
    } else if (order == "YZX") {
        index_order = {1,2,0};
    } else if (order == "ZXY") {
        index_order = {2,0,1};
    } else if (order == "ZYX") {
        index_order = {2,1,0};
    } else {
        throw std::invalid_argument("Invalid Euler angle order");
    }

    return as_euler(index_order, degrees);
}


// Static method to create a Rotation object from Euler angles
Rotation Rotation::from_euler(std::string order, std::vector<double> values, bool degrees) { //TODO: Unsure if this is right

    if(order.length() != values.size()){
        throw std::invalid_argument("Number of provided axis from input string " + order + " : " + std::to_string(order.length()) + " does not match number of provided angles " + std::to_string(values.size()));
    }

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
    } else if (order == "YXZ") {
        angles[0] = values[1];   // Pitch
        angles[1] = values[0];   // Roll
        angles[2] = values[2];   // Yaw
    } else if (order == "YZX") {
        angles[0] = values[2];   // Pitch
        angles[1] = values[0];   // Yaw
        angles[2] = values[1];   // Roll
    } else if (order == "ZXY") {
        angles[0] = values[1];   // Yaw
        angles[1] = values[2];   // Roll
        angles[2] = values[0];   // Pitch
    } else if (order == "ZYX") {
        angles[0] = values[2];   // Yaw
        angles[1] = values[1];   // Pitch
        angles[2] = values[0];   // Roll
    } else if (order == "X") {
        angles[0] = values[0];   // Yaw
    } else if (order == "Y") {
        angles[1] = values[0];   // Pitch
    } else if (order == "Z") {
        angles[2] = values[0];   // Roll
    } else {
        throw std::invalid_argument("Invalid Euler angle order");
    }

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

    std::array<double, 4> q;


    q[3] = cr * cp * cy + sr * sp * sy; //w
    q[0] = sr * cp * cy - cr * sp * sy; //x
    q[1] = cr * sp * cy + sr * cp * sy; //y
    q[2] = cr * cp * sy - sr * sp * cy; //z

    return Rotation(q);
}



// Helper function to normalize a quaternion
void Rotation::normalize_quaternion(std::array<double, 4>& q) {
    double norm_squared = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (norm_squared == 0.0) {
        throw std::runtime_error("Attempted to normalize a zero quaternion");
    }
    double norm = std::sqrt(norm_squared);
    q = {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
    return;
}



// Static method to create a Rotation object from a quaternion
Rotation Rotation::from_quat(std::array<double, 4> q, bool scalar_last) {
    if(scalar_last == false){
        q = {q[1], q[2], q[3], q[0]};
    }
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
    return from_quat({0,0,0,1});
}

bool Rotation::operator==(const Rotation& obj) const{
    std::array<double, 4> q1 = _q;
    std::array<double, 4> q2 = obj.as_quat();

    bool fequal = true;
    for(int i = 0; i<4; i++){
        if(q1[i] != q2[i]){
            fequal = false;
        }
    }
    return fequal;
}
