#ifndef POSE_HPP
#define POSE_HPP

#include <array>
#include "Rotation.hpp" // Include the Rotation class definition

class Pose {
private:
    Rotation orientation;
    std::array<double, 3> position;

public:
    Pose(std::array<double, 3> position = {0, 0, 0}, Rotation orientation = Rotation::identity());
    bool operator==(const Pose& object) const;
    void set_heading_from_isaac_quat(std::array<double, 4> quat_s_last);
    std::array<double, 4> get_quat_scalar_first() const;
    std::array<double, 4> get_quat_scalar_last() const;
    void set_heading_from_angle(double angle, bool degrees = true);
    void set_heading_to_destination(const std::array<double, 3>& dest);
    void set_heading_from_origin(const std::array<double, 3>& origin);
    void randomize_orientation();
    double get_heading_from_orientation() const;
    Rotation get_orientation() const;
    std::array<double, 3> get_position() const;
    void set_orientation(Rotation f);
    void set_position(std::array<double, 3>);
};

#endif // POSE_HPP
