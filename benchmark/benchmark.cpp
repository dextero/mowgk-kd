#include <fenv.h>
#include <iostream>
#include <string>
#include <cmath>

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

int main(int /*argc*/,
         char* /*argv*/[])
{
    feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW);

    std::unique_ptr<kd_tree<double>> tree;

    // volume represented by the tree
    Bbox_3 kd_tree_box = {-1, -1, -1, 1, 1, 1};
    // approximated function
    auto function = [](double x, double y, double z) {
        return exp(-(x*x+y*y+z*z)/(2*0.1));
    };
    // maximum allowed error. Note: actual error depends on the accuracy
    // of the ErrorEstimator template parameter of the kd_tree.
    double required_accuracy = 0.2;

    typedef decltype(function(0,0,0)) ElementT;

    // tree construction
    tree = kd_tree<ElementT,
                   half_box_splitter<ElementT>
                  >::build(kd_tree_box, function, required_accuracy);

    printf("%ld\n", count_nodes(tree));
    std::pair<double, double> balance = balance_factor(tree);
    printf("%f %f\n", balance.first, balance.second);
    std::pair<double, double> error = tree_error(tree, function);
    printf("%f %f\n", error.first, error.second);

    tree = kd_tree<ElementT,
                   gradient_box_splitter<ElementT, 10>
                  >::build(kd_tree_box, function, required_accuracy);

    printf("%ld\n", count_nodes(tree));
    balance = balance_factor(tree);
    printf("%f %f\n", balance.first, balance.second);
    error = tree_error(tree, function);
    printf("%f %f\n", error.first, error.second);

    return 0;
}
