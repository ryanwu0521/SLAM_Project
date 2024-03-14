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
    Rotation(std::array<double, 4> q = {0,0,0,1});
    ~Rotation();
    std::array<double, 4> as_quat() const;
    std::array<double, 3> as_euler(std::string order = "xyz", bool degrees = false) const;
    static Rotation from_euler(std::string order, std::vector<double> values, bool degrees = false);
    static Rotation identity();
    static Rotation from_quat(std::array<double, 4> q);
    static std::array<double, 4> normalize_quaternion(const std::array<double, 4>& q);
    static std::array<double, 4> multiply_quaternions(const std::array<double, 4>& q1, const std::array<double, 4>& q2);
};

#endif // ROTATION_HPP
