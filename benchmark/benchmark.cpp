#include <fenv.h>
#include <iostream>
#include <string>
#include <cmath>

// CGAL assertions fail under valgrind, disable them for debugging
#ifdef _DEBUG
#   define CGAL_NO_ASSERTIONS
#endif

#include <CGAL/squared_distance_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Line_3.h>

// HACK: copied imports from kd_tree.h to suppress 'redeclared with different access' exception
#include <functional>
#include <memory>
#include <cassert>

#include <CGAL/Cartesian.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Bbox_3.h>

// HACK: allow kd-tree structure inspection
#define private public
#include "kd_tree.h"
#undef private

struct ScopedTimer {
    std::function<void(double diff_s)> at_scope_exit;
    struct timespec start;

    ScopedTimer(const std::function<void()>& at_scope_enter,
                const std::function<void(double diff_s)>& at_scope_exit):
        at_scope_exit(at_scope_exit)
    {
        if (at_scope_enter) {
            at_scope_enter();
        }
        clock_gettime(CLOCK_REALTIME, &start);
    }

    ScopedTimer(const std::function<void(double diff_s)>& at_scope_exit):
        ScopedTimer({}, at_scope_exit)
    {}

    ScopedTimer(const std::string& msg):
        ScopedTimer([msg]() { fprintf(stderr, "%s: start\n", msg.c_str()); },
                    [msg](double time_s) { fprintf(stderr, "%s: %f s\n", msg.c_str(), time_s); })
    {}

    ~ScopedTimer()
    {
        struct timespec end;
        clock_gettime(CLOCK_REALTIME, &end);

        long diff_ns = (end.tv_sec - start.tv_sec) * 10e9 + (end.tv_nsec - start.tv_nsec);
        double diff_s = (double)diff_ns / 10e9;

        if (at_scope_exit) {
            at_scope_exit(diff_s);
        }
    }
};

void print_tree(std::unique_ptr<kd_tree<double>> const &tree, size_t level = 0, size_t offset = 0) {
    if (!tree->is_leaf) {
        printf("%ld %ld\n%ld %ld\n\n", level, offset, level+1, offset*2);
        printf("%ld %ld\n%ld %ld\n\n", level, offset, level+1, offset*2+1);
        print_tree(tree->data.node.low, level+1, offset*2);
        print_tree(tree->data.node.high, level+1, offset*2+1);
    }
}

size_t count_nodes(std::unique_ptr<kd_tree<double>> const &tree) {
    size_t count = 1;
    if (!tree->is_leaf) {
        count += count_nodes(tree->data.node.low);
        count += count_nodes(tree->data.node.high);
    }
    return count;
}

std::pair<double, double> balance_factor(std::unique_ptr<kd_tree<double>> const &tree) {
    ScopedTimer timer("balance_factor");

    size_t sum = 0;
    size_t sum_sq = 0;
    size_t count = 0;

    {
        std::vector<kd_tree<double>*> stack_tree;
        std::vector<size_t> stack_level;
        stack_tree.push_back(tree.get());
        stack_level.push_back(0);

        while (stack_tree.size() > 0) {
            assert(stack_tree.size() == stack_level.size());

            kd_tree<double> *curr = stack_tree.back();
            size_t level = stack_level.back();
            stack_tree.pop_back();
            stack_level.pop_back();

            if (curr->is_leaf) {
                sum += level;
                sum_sq += level * level;
                count++;
            } else {
                stack_tree.push_back(curr->data.node.low.get());
                stack_level.push_back(level+1);

                stack_tree.push_back(curr->data.node.high.get());
                stack_level.push_back(level+1);
            }
        }
    }

    double m = 1. * sum / count;
    double stdev = sqrt(1. * sum_sq / count - m * m);

    return std::make_pair(m, stdev);
}

std::vector<Vector_3> make_grid(size_t samples_per_axis) {
    ScopedTimer timer("make_grid");

    std::vector<Vector_3> positions;
    positions.reserve(samples_per_axis * samples_per_axis * samples_per_axis);

    std::pair<double, double> x_range(-1,1);
    std::pair<double, double> y_range(-1,1);
    std::pair<double, double> z_range(-1,1);

    for (size_t i = 0; i < samples_per_axis * samples_per_axis * samples_per_axis; i++) {
        size_t i_x = i / (samples_per_axis * samples_per_axis);
        size_t i_y = (i / samples_per_axis) % samples_per_axis;
        size_t i_z = i % samples_per_axis;

        double x = x_range.first + (1. * i_x / (samples_per_axis - 1)) * (x_range.second - x_range.first);
        double y = y_range.first + (1. * i_y / (samples_per_axis - 1)) * (y_range.second - y_range.first);
        double z = z_range.first + (1. * i_z / (samples_per_axis - 1)) * (z_range.second - z_range.first);

        positions.emplace_back(x, y, z);
    }

    return positions;
}

double measure_average_access_time_us(const std::unique_ptr<kd_tree<double>> &tree,
                                      const std::vector<Vector_3> &positions,
                                      std::vector<double> &out_approximate_values) {
    ScopedTimer timer("measure_average_access_time_us");

    constexpr size_t REPS = 100;

    double total_time_s = 0.0;
    out_approximate_values.clear();
    out_approximate_values.resize(positions.size());

    {
        ScopedTimer timer([&total_time_s](double time_s) { total_time_s = time_s; });

        for (size_t rep = 0; rep < REPS; ++rep) {
            for (size_t i = 0; i < positions.size(); ++i) {
                const Vector_3 &p = positions[i];
                out_approximate_values[i] = tree->value_at(p.x(), p.y(), p.z());
            }
        }
    }

    return total_time_s * 1000000.0 / (double)(positions.size() * REPS);
}

std::pair<double, double> tree_error(const std::function<double(double x, double y, double z)> &func,
                                     const std::vector<Vector_3> &positions,
                                     const std::vector<double> approximate_values) {
    assert(positions.size() == approximate_values.size());
    ScopedTimer timer("tree_error");

    double sum = 0.;
    double max_err = 0.;

    for (size_t i = 0; i < positions.size(); ++i) {
        const Vector_3& p = positions[i];
        double err = std::abs(func(p.x(), p.y(), p.z()) - approximate_values[i]);
        if (err > max_err) max_err = err;
        sum += err;
    }

    double m = 1. * sum / (double)positions.size();

    return std::make_pair(max_err, m);
}

typedef CGAL::Point_3<Kernel> Point_3;
typedef CGAL::Line_3<Kernel> Line_3;

Function3D<double> get_function(size_t id) {
    switch (id) {
        case 0:
            // a gauss function around the (0,0,0) with 1-sigma equal to 0.1
            return [](double x, double y, double z) {
                return exp(-(x*x+y*y+z*z)/(2*0.1));
            };
        case 1:
            // proportional to distance from a line
            return [](double x, double y, double z) {
                Point_3 point(x, y, z);
                Point_3 p(-1., -1., -0.5);
                Point_3 q( 0.5, 1.,  0.);
                Line_3 line(p, q);

                return CGAL::squared_distance<Kernel>(point, line);
            };
        case 2:
            // proportional to distance from three points
            return [](double x, double y, double z) {
                Point_3 point(x, y, z);
                Point_3 p( 0.5,  0.5,  0.5);
                Point_3 q( 0.5,  0.5, -0.5);
                Point_3 r( 0.0,  0.0,  0.0);

                return (sqrt(CGAL::squared_distance<Kernel>(point, p)) +
                       sqrt(CGAL::squared_distance<Kernel>(point, q)) +
                       sqrt(CGAL::squared_distance<Kernel>(point, r))) / 3;
            };
        case 3:
            // Schwefel's function
            return [](double x, double y, double z) {
                double s = 0;

                s += -x * sin(sqrt(std::abs(x*500)));
                s += -y * sin(sqrt(std::abs(y*500)));
                s += -z * sin(sqrt(std::abs(z*500)));

                return s / 3;
            };
        default:
            // constant zero function
            return [](double, double, double) {
                return 0;
            };
    }
}

int main(int argc,
         char* argv[])
{
    if (argc != 3) {
        printf("Usage: bin/benchmark <fun_from> <fun_to>\n");
        exit(1);
    }

    size_t from = atoi(argv[1]);
    size_t to = atoi(argv[2]);

    assert(from <= to);
    assert(to <= 3);

    feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW);

    std::unique_ptr<kd_tree<double>> tree;

    // volume represented by the tree
    Bbox_3 kd_tree_box = {-1, -1, -1, 1, 1, 1};

    std::vector<double> tolerance = {1, .9, .8, .7, .6, 0.5, 0.4, 0.3, 0.2, 0.1};
    constexpr size_t GRID_POINTS_PER_AXIS = 101;

    fprintf(stderr, "splitter build_time_s access_time_us.mean fun_i tolerance node_count balance.mean balance.stdev error.max error.mean\n");
    for (size_t fun_i = from; fun_i <= to; fun_i++) {
        Function3D<double> fun = get_function(fun_i);
        std::for_each(tolerance.begin(), tolerance.end(), [&](const double tolerance) {
            double build_time_s;

            {
                ScopedTimer timer([&build_time_s](double time_s) { build_time_s = time_s; });
                tree = kd_tree<double,
                               half_box_splitter<double>
                              >::build(kd_tree_box, fun, tolerance);
            }

            size_t node_count = count_nodes(tree);
            std::pair<double, double> balance = balance_factor(tree);
            std::vector<Vector_3> grid = make_grid(GRID_POINTS_PER_AXIS);
            std::vector<double> approximate_values;
            double average_access_time_us = measure_average_access_time_us(tree, grid, approximate_values);
            std::pair<double, double> error = tree_error(fun, grid, approximate_values);

            printf("half %ld %f %f %f %ld %f %f %f %f\n",
                fun_i,
                tolerance,
                build_time_s,
                average_access_time_us,
                node_count,
                balance.first,
                balance.second,
                error.first,
                error.second);

            {
                ScopedTimer timer([&build_time_s](double time_s) { build_time_s = time_s; });
                tree = kd_tree<double,
                               gradient_box_splitter<double, 10>
                              >::build(kd_tree_box, fun, tolerance);
            }

            node_count = count_nodes(tree);
            balance = balance_factor(tree);
            average_access_time_us = measure_average_access_time_us(tree, grid, approximate_values);
            error = tree_error(fun, grid, approximate_values);

            printf("grad %ld %f %f %f %ld %f %f %f %f\n",
                fun_i,
                tolerance,
                build_time_s,
                average_access_time_us,
                node_count,
                balance.first,
                balance.second,
                error.first,
                error.second);
        });
    }

    return 0;
}
