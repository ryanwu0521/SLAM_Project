#ifndef ROTATION_HPP
#define ROTATION_HPP

#include<array>
#include<string>
#include<vector>

class Rotation
{
private:
    std::array<double, 4> _q; //Uses scalar last convention

public:
    Rotation(std::array<double, 4> q);
    ~Rotation();
    std::array<double, 4> as_quat(bool scalar_last = true) const;
    std::array<double, 3> as_euler(std::string order = "XYZ", bool degrees = false) const;
    std::array<double, 3> as_euler(const std::array<int, 3>& order, bool degrees = false) const;
    std::array<std::array<double, 3>, 3> as_matrix() const;
    static Rotation from_euler(std::string order, std::vector<double> values, bool degrees = false);
    static Rotation identity();
    static Rotation from_quat(std::array<double, 4> q, bool scalar_last = true);

    static void normalize_quaternion(std::array<double, 4>& q);
    static std::array<double, 4> multiply_quaternions(const std::array<double, 4>& q1, const std::array<double, 4>& q2);

    bool operator==(const Rotation& obj) const;
};

#endif // ROTATION_HPP
