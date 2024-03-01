#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <thread>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)
#define M_PI           3.14159265358979323846

double  _z_hit = .6;
double  _z_short = .19;
double  _z_max = .01;
double  _z_rand = .20;
double _sigma_hit = 20;
double _lambda_short = .02;
double _max_range = 1000;


namespace py = pybind11;

double p_rand(double z_t_k) {
    if ((z_t_k >= 0 and z_t_k <= _max_range)) {
        return  1 / _max_range;
    }
    else {
        return 0;
    }
}

double p_max(double z_t_k) {
    if (((z_t_k >= (_max_range - 1)) and z_t_k <= _max_range)) {
        return 1;
    }
    else {
        return 0;
    }
}

double p_short(double z_t_k, double z_t_true) {
    if ((z_t_k >= 0 and z_t_k <= z_t_true)) {
        double pshort = _lambda_short * exp(-(_lambda_short * z_t_k));
        double eta = 1 / (1 - exp(-(_lambda_short * z_t_true)));
        return  eta * pshort;
    }
    else {
        return 0;
    }
}

double p_hit(double z_t_k, double z_t_true) {
    if ((z_t_k >= 0 and z_t_k <= _max_range)) {
        double numerator = exp(-0.5 * pow(((z_t_k - z_t_true) / _sigma_hit), 2));
        return numerator / (sqrt(2 * M_PI) * _sigma_hit);
        }
    else {
        return 0;
    }
}

void array_test(py::array test) {
    py::buffer_info info = test.request();
    auto ptr = static_cast<double*>(info.ptr);
    int n = 1;
    for (auto r : info.shape) {
        n *= r;
    }

    for (int i = 0; i < n; i++) {
        *ptr *= 2;
        ptr++;
    }

    return;
}

struct Index {
    int x, y;
};

Index get_coords(double x, double y) {
    Index idx;
    idx.x = floor(x / 10);
    idx.y = floor(y / 10);
    return idx;
}

double get_map_value(double* ptr, Index idx, std::vector<py::ssize_t> strides) {
    double value = *(ptr + (strides[0]/8 * idx.y) + (strides[1]/8 * idx.x));
    return value;
}

double ray_casting_exact(py::array occupancy_map, double x, double y, double th) {
    py::buffer_info occupancy_map_info = occupancy_map.request();
    auto occupancy_map_ptr = static_cast<double*>(occupancy_map_info.ptr);
    int map_size = 800 * 800;
    
    int occupancy_map_inc = 1;
    for (auto r : occupancy_map_info.shape) {
        occupancy_map_inc *= r;
    }

    double sinth = sin(th);
    double costh = cos(th);

    int  dirSignX = costh > 0 and 1 or 0;
    int  dirSignY = sinth > 0 and 1 or 0;

    double xc = x;
    double yc = y;
    double d = 0;
    double map_value;
    double dd_x;
    double dd_y;
    Index idxc;

    while (true) {
        idxc = get_coords(xc, yc);
        if (idxc.x < 0 or idxc.y < 0 or idxc.x > 799 or idxc.y > 799) {
            return -2;
        }
        map_value = get_map_value(occupancy_map_ptr, idxc, occupancy_map_info.strides);
        if (map_value == -1) {
            return -1;
        }
        else if (map_value > 0.35) {
            return d;
        }

        if (sinth == 0) {
            d += (((idxc.x + dirSignX) * 10) - xc) / costh;
        }
        else if (costh == 0) {
            d += ((idxc.y + dirSignY) * 10 - yc) / sinth;
        }
        else {
            dd_x = ((idxc.x + dirSignX) * 10 - xc) / costh;
            dd_y = ((idxc.y + dirSignY) * 10 - yc) / sinth;
            if (dd_x < dd_y) {
                d += dd_x;
            }
            else {
                d += dd_y;
            }
        }

        d += .001;
        xc = x + costh * d;
        yc = y + sinth * d;
    }
}

double test_map(py::array occupancy_map, int ix, int iy) {
    py::buffer_info occ_info = occupancy_map.request();
    double* map_ptr = static_cast<double*>(occ_info.ptr);
    return *(map_ptr + occ_info.strides[0]*ix/8 + occ_info.strides[1]*iy/8);
}

void calculate_likelihood(py::array meas_laser, py::array raycast_laser, py::array output) {
    py::buffer_info meas_laser_info = meas_laser.request();
    py::buffer_info raycast_laser_info = raycast_laser.request();
    py::buffer_info output_info = output.request();

    auto meas_laser_ptr = static_cast<double*>(meas_laser_info.ptr);
    auto raycast_laser_ptr = static_cast<double*>(raycast_laser_info.ptr);
    auto output_ptr = static_cast<double*>(output_info.ptr);

    int strm = meas_laser_info.strides[0] / 8;

    int strrx = raycast_laser_info.strides[0] / 8;
    int strry = raycast_laser_info.strides[1] / 8;

    int stro = output_info.strides[0] / 8;

    double raycast_c;
    double meas_c;

    double p_c;

    double total_log_likelihood;
    double min_log_likelihood = 0;

    for (int point_idx = 0; point_idx < raycast_laser_info.shape[0]; point_idx++) {
        total_log_likelihood = 0;
        for (int laser_idx = 0; laser_idx < raycast_laser_info.shape[1]; laser_idx++) {
            raycast_c = *(raycast_laser_ptr + strrx * point_idx + strry * laser_idx);
            meas_c = *(meas_laser_ptr + strm * laser_idx);
            if ((raycast_c < 0 or meas_c < 0)) {
                p_c = 1/_max_range;
            }
            else if (raycast_c > _max_range) {
                p_c = _z_short * p_short(meas_c, raycast_c) + (_z_hit + _z_max)*p_max(meas_c) + _z_rand*p_rand(meas_c);
            }
            else {
                p_c = _z_hit*p_hit(meas_c, raycast_c) + _z_short * p_short(meas_c, raycast_c) + _z_max * p_max(meas_c) + _z_rand * p_rand(meas_c);
            }
            total_log_likelihood += log10(p_c)/180;
        }
        if (total_log_likelihood < min_log_likelihood) {
            min_log_likelihood = total_log_likelihood;
        }
        *(output_ptr + stro * point_idx) = total_log_likelihood;
    }

    double temp;
    double sum_likelihood = 0;

    for (int point_idx = 0; point_idx < raycast_laser_info.shape[0]; point_idx++) {
        temp = *(output_ptr + stro * point_idx);
        *(output_ptr + stro * point_idx) = pow(10,(temp-min_log_likelihood));
        sum_likelihood += *(output_ptr + stro * point_idx);
    }

    for (int point_idx = 0; point_idx < raycast_laser_info.shape[0]; point_idx++) {
        temp = *(output_ptr + stro * point_idx);
        *(output_ptr + stro * point_idx) = temp/sum_likelihood;
    }
    return;
}

void ray_casting_exact_all(py::array occupancy_map, py::array X_bar, py::array output_arr) {
    py::buffer_info X_bar_info      = X_bar.request();
    py::buffer_info output_arr_info = output_arr.request();

    //int n_threads = std::thread::hardware_concurrency();
    //std::cout << n_threads;

    auto X_bar_ptr                  = static_cast<double*>(X_bar_info.ptr);
    auto output_arr_ptr             = static_cast<double*>(output_arr_info.ptr);
    
    double angle_inc = M_PI / output_arr_info.shape[1];

    if (output_arr_info.shape[0] != X_bar_info.shape[0]) {
        throw std::invalid_argument("Dimension mismatch");
    }

    double x;
    double y;
    double th;
    double d;
    
    int strxx = X_bar_info.strides[0] / 8;
    int strxy = X_bar_info.strides[1] / 8;
    
    int strox = output_arr_info.strides[0] / 8;
    int stroy = output_arr_info.strides[1] / 8;

    for (int X_bar_idx = 0; X_bar_idx < X_bar_info.shape[0]; X_bar_idx++) {

        x = *(X_bar_ptr);
        y = *(X_bar_ptr + strxy);
        th = *(X_bar_ptr + (2 * strxy)) - M_PI/2;

        for (int laser_num = 0; laser_num < 180; laser_num++) {

            *(output_arr_ptr+stroy*laser_num+strox*X_bar_idx) = ray_casting_exact(occupancy_map, x, y, th);
            th += angle_inc;
        }

        X_bar_ptr += strxx;

    }

    return;
}


PYBIND11_MODULE(cpp_mcl_acceleration, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: python_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";
    m.def("p_rand", &p_rand);
    m.def("p_max", &p_max);
    m.def("p_short", &p_short);
    m.def("p_hit", &p_hit);
    m.def("calculate_likelihood", &calculate_likelihood);

    m.def("test_map", &test_map);
    m.def("ray_casting_exact_all", &ray_casting_exact_all);
    m.def("ray_casting_exact", &ray_casting_exact);
    m.def("array_test",&array_test, R"pbdoc(
        Test for the arrays by doubling it
    )pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}