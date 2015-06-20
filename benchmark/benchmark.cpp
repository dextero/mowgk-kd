#include <fenv.h>
#include <iostream>
#include <string>
#include <cmath>

#include <CGAL/squared_distance_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Line_3.h>

// HACK: copied imports from kd_tree.h to suppress 'redeclared with different access' exception
#include <functional>
#include <memory>
#include <cassert>

// CGAL assertions fail under valgrind, disable them for debugging
#ifdef _DEBUG
#   define CGAL_NO_ASSERTIONS
#endif

#include <CGAL/Cartesian.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Bbox_3.h>

// HACK: allow kd-tree structure inspection
#define private public
#include "kd_tree.h"
#undef private

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

std::pair<double, double> tree_error(std::unique_ptr<kd_tree<double>> const &tree,
        const std::function<double(double x, double y, double z)> &func,
        size_t samples = 101) {
    double sum = 0.;
    double max_err = 0.;

    {
        std::pair<double, double> x_range(-1,1);
        std::pair<double, double> y_range(-1,1);
        std::pair<double, double> z_range(-1,1);

        for (size_t i = 0; i < samples * samples * samples; i++) {
            size_t i_x = i / (samples * samples);
            size_t i_y = (i / samples) % samples;
            size_t i_z = i % samples;

            double x = x_range.first + (1. * i_x / (samples - 1)) * (x_range.second - x_range.first);
            double y = y_range.first + (1. * i_y / (samples - 1)) * (y_range.second - y_range.first);
            double z = z_range.first + (1. * i_z / (samples - 1)) * (z_range.second - z_range.first);

            double err = std::abs(func(x,y,z) - tree->value_at(x,y,z));
            if (err > max_err) max_err = err;
            sum += err;
        }
    }

    double m = 1. * sum / samples / samples / samples;

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

                return sqrt(CGAL::squared_distance<Kernel>(point, line));
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

    std::vector<double> tolerance = {1, .9, .8, .7, .6, 0.5, 0.4, 0.3, 0.2};

    for (size_t fun_i = from; fun_i <= to; fun_i++) {
        Function3D<double> fun = get_function(fun_i);
        std::for_each(tolerance.begin(), tolerance.end(), [&](const double tolerance) {
            tree = kd_tree<double,
                           half_box_splitter<double>
                          >::build(kd_tree_box, fun, tolerance);
            size_t node_count = count_nodes(tree);
            std::pair<double, double> balance = balance_factor(tree);
            std::pair<double, double> error = tree_error(tree, fun);

            printf("half %ld %f %ld %f %f %f %f\n",
                fun_i,
                tolerance,
                node_count,
                balance.first,
                balance.second,
                error.first,
                error.second);


            tree = kd_tree<double,
                           gradient_box_splitter<double, 10>
                          >::build(kd_tree_box, fun, tolerance);

            node_count = count_nodes(tree);
            balance = balance_factor(tree);
            error = tree_error(tree, fun);

            printf("grad %ld %f %ld %f %f %f %f\n",
                fun_i,
                tolerance,
                node_count,
                balance.first,
                balance.second,
                error.first,
                error.second);
        });
    }

    return 0;
}
