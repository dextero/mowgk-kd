KD-tree based 3D function approximator
======================================

A student project for the Object Modeling in Computer Graphics class in the AGH University of Science and Technology, written by Marcin Radomski and Jakub Sawicki.

The implemented class approximates given 3D function with an arbitrary precision\* using a KD-tree. The code is highly customizable - it allows the user to supply a custom error estimator and tree node split policy.

\* precision depends on accuracy of supplied error estimator.

Installation
------------

The projects consists of a single header file (include/kd\_tree.h), but requires the CGAL library to work.

* super-quick installation (to /usr/local/include):
```bash
$ sudo make install
```

* install to custom directory:
```bash
$ mkdir build && cd build
$ cmake -DCMAKE_INSTALL_PREFIX=<your_install_path> ..
$ make install
```

Tree visualizer
---------------

The project contains an example app that shows the structure of constructed kd-tree. To run it, use:

```bash
$ make visualizer
$ ./bin/kd
```

To install required libraries under Ubuntu:

```bash
$ sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev libglew-dev libx11-dev libdevil-dev libassimp-dev
```

Example usage:
--------------

```c++
// volume represented by the tree
Bbox_3 kd_tree_box = {-1, -1, -1, 1, 1, 1};
// approximated function
auto function = [](double x, double y, double z) {
    return exp(-(x*x+y*y+z*z)/(2*0.1));
};
// maximum allowed error
double required_accuracy = 0.2;

// construction with default box splitter and error estimator
std::unique_ptr<kd_tree<double>> tree = kd_tree<double>::build(kd_tree_box, function, required_accuracy);

// construction using custom box splitter/error estimator
using BoxSplitter = gradient_box_splitter<double, 10>;
using ErrorEstimator = sampling_scalar_error_estimator<3>;

using KdTree = kd_tree<double, BoxSplitter, ErrorEstimator>;

// tree construction
std::unique_ptr<KdTree> tree = KdTree::build(kd_tree_box, function, required_accuracy);

// tree sampling
double approximate_value = tree->value_at(x, y, z);
```
