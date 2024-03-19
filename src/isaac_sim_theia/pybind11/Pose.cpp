#include "include/Pose.hpp"
#include <cmath> // For std::isnan
#include <stdexcept> // For std::runtime_error
#include <cstdlib> // For rand

Pose::Pose(std::array<double, 3> position, Rotation orientation)
    : position(position), orientation(orientation) {
    std::array<double, 4> quat = orientation.as_quat();
    if (std::isnan(quat[0]) || std::isnan(quat[1]) || std::isnan(quat[2]) || std::isnan(quat[3])) {
        throw std::runtime_error("Tried to initialize to NaN");
    }
}

bool Pose::operator==(const Pose& object) const {
    return (position[0] == object.position[0] && position[1] == object.position[1] && get_heading_from_orientation() == object.get_heading_from_orientation());
}

void Pose::set_heading_from_isaac_quat(std::array<double, 4> quat_s_last) {
    orientation = Rotation::from_quat(quat_s_last);
}

std::array<double, 4> Pose::get_quat_scalar_first() const {
    return orientation.as_quat(false);
}

std::array<double, 4> Pose::get_quat_scalar_last() const {
    return orientation.as_quat();
}

void Pose::set_heading_from_angle(double angle, bool degrees) {
    orientation = Rotation::from_euler("Z", {angle}, degrees);
}

void Pose::set_heading_to_destination(const std::array<double, 3>& dest) {
    double delta_x = dest[0] - position[0];
    double delta_y = dest[1] - position[1];
    double angle = std::atan2(delta_y, delta_x);
    orientation = Rotation::from_euler("Z", {angle}, false);
}

void Pose::set_heading_from_origin(const std::array<double, 3>& origin) {
    double delta_x = position[0] - origin[0];
    double delta_y = position[1] - origin[1];
    double angle = std::atan2(delta_y, delta_x);
    orientation = Rotation::from_euler("Z", {angle}, false);
}

void Pose::randomize_orientation() {
    std::array<double, 4> quat;
    double angle = rand() % 361;
    orientation = set_heading_from_angle(angle, true);
}

double Pose::get_heading_from_orientation() const {
    std::array<double, 3> euler_angles = orientation.as_euler("XYZ", true);
    return euler_angles[2];
}

Rotation Pose::get_orientation() const{
    return orientation;
}

std::array<double, 3> Pose::get_position() const{
    return position;
}

void Pose::set_orientation(Rotation f) {
    orientation = f;
    return;
}

void Pose::set_position(std::array<double, 3> f) {
    position = f;
    return;
}