#include "include/Trajectory.hpp"
#include <cmath>

void Trajectory::_calculate_times() {
    double delta_angle = end_pose.get_heading_from_orientation() - start_pose.get_heading_from_orientation();
    while (delta_angle < -180) {
        delta_angle += 360;
    }
    while (delta_angle > 180) {
        delta_angle -= 360;
    }

    turn_time = start_time + std::abs(delta_angle) / angular_speed + wait_time;
    end_time = turn_time + std::sqrt(std::pow(end_pose.get_position()[0] - start_pose.get_position()[0], 2) +
                                     std::pow(end_pose.get_position()[1] - start_pose.get_position()[1], 2) +
                                     std::pow(end_pose.get_position()[2] - start_pose.get_position()[2], 2)) / linear_speed;
}

void Trajectory::_setup_slerp() {
    std::vector<Rotation> key_rots = {start_pose.get_orientation(), start_pose.get_orientation(),
                                       end_pose.get_orientation(), end_pose.get_orientation()};
    std::vector<double> key_times = {start_time, start_time + wait_time + 0.001,
                                     turn_time + 0.002, end_time + 0.003}; // hacky bump off
    slerp = std::make_unique<Slerp>(key_rots, key_times);
}

Trajectory::Trajectory(Pose start_pose, Pose end_pose, double linear_speed, double angular_speed, double start_time, double wait_time)
    : start_pose(start_pose), end_pose(end_pose), linear_speed(linear_speed), angular_speed(angular_speed), start_time(start_time), wait_time(wait_time){
    _calculate_times();
    _setup_slerp();
}

Pose Trajectory::get_pose_at_time(double time) {

    if (time <= start_time) {
        return start_pose;
    }
    if (time >= end_time) {
        return end_pose;
    }

    if (time < turn_time) {
        return Pose(start_pose.get_position(), (*slerp)(time));
    }

    std::array<double, 3> fposition;
    if (end_time <= turn_time && time >= turn_time) {
        fposition = end_pose.get_position();
    } else {
        double t = (time - turn_time) / (end_time - turn_time);
        fposition[0] = t * (end_pose.get_position()[0] - start_pose.get_position()[0]) + start_pose.get_position()[0];
        fposition[1] = t * (end_pose.get_position()[1] - start_pose.get_position()[1]) + start_pose.get_position()[1];
        fposition[2] = t * (end_pose.get_position()[2] - start_pose.get_position()[2]) + start_pose.get_position()[2];
    }

    return Pose(fposition, (*slerp)(time));
}

bool Trajectory::is_finished(double time) {
    return time > end_time;
}
