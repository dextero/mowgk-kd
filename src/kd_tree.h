#include <functional>
#include <memory>
#include <cassert>

#ifdef _DEBUG
#   define CGAL_NO_ASSERTIONS
#endif

#include <CGAL/Cartesian.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Bbox_3.h>

typedef CGAL::Cartesian<double> Kernel;
typedef Kernel::Point_3 Vector_3;

template<typename T>
using Function3D = std::function<T(double x, double y, double z)>;

enum class Axis { X = 0, Y = 1, Z = 2 };

constexpr double EPSILON = 1e-5;

class Bbox_3: public CGAL::Bbox_3
{
public:
    inline Bbox_3():
        Bbox_3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    {}

    inline Bbox_3(double xmin,
                  double ymin,
                  double zmin,
                  double xmax,
                  double ymax,
                  double zmax):
        CGAL::Bbox_3(xmin, ymin, zmin, xmax, ymax, zmax)
    {}

    inline Vector_3 center() const {
        return { (xmin() + xmax()) / 2.0,
                 (ymin() + ymax()) / 2.0,
                 (zmin() + zmax()) / 2.0 };
    }

    inline double size(Axis axis) const {
        return max((int)axis) - min((int)axis);
    }

    inline Vector_3 size() const {
        return { xmax() - xmin(),
                 ymax() - ymin(),
                 zmax() - zmin() };
    }

    inline Axis get_longest_axis() const {
        double sizes[] = {
            size(Axis::X),
            size(Axis::Y),
            size(Axis::Z)
        };

        if (sizes[0] > sizes[1]) {
            if (sizes[0] > sizes[2]) {
                return Axis::X;
            } else {
                return Axis::Z;
            }
        } else if (sizes[1] > sizes[2]) {
            return Axis::Y;
        } else {
            return Axis::Z;
        }
    }

    inline double half(Axis axis) const {
        return (max((int)axis) + min((int)axis)) / 2.0;
    }

    inline double xhalf() const {
        return xmin() + size(Axis::X) / 2.0;
    }
    inline double yhalf() const {
        return ymin() + size(Axis::Y) / 2.0;
    }
    inline double zhalf() const {
        return zmin() + size(Axis::Z) / 2.0;
    }

    inline bool contains(double x, double y, double z) const
    {
        return xmin() <= x && x <= xmax()
            && ymin() <= y && y <= ymax()
            && zmin() <= z && z <= zmax();
    }
};

struct BoxSplit
{
    double pos;
    Axis axis;
};

class Bbox_3;
typedef std::function<BoxSplit(const Bbox_3 &bounding_box)> BoxSplitter;

template<typename ElementT>
struct half_box_splitter
{
    static BoxSplit split(const Bbox_3 &bb,
                          const Function3D<ElementT> &)
    {
        Axis split_axis = bb.get_longest_axis();

        return {
            .pos = bb.half(split_axis),
            .axis = split_axis
        };
    }
};

template<typename ElementT, size_t SamplesSize>
struct gradient_box_splitter
{
    static BoxSplit split(const Bbox_3 &bb,
                          const Function3D<ElementT> &fun)
    {
        ElementT sum[] = {0, 0, 0};
        //printf("[%.3f - %.3f] x ", bb.xmax(), bb.xmin());
        //printf("[%.3f - %.3f] x ", bb.ymax(), bb.ymin());
        //printf("[%.3f - %.3f]\n", bb.zmax(), bb.zmin());
        double samplesSize = (double) SamplesSize;
        double step_x = (bb.xmax() - bb.xmin()) / samplesSize,
               step_y = (bb.ymax() - bb.ymin()) / samplesSize,
               step_z = (bb.zmax() - bb.zmin()) / samplesSize;
        //printf("step_x = %.3f\n", step_x);
        //printf("step_y = %.3f\n", step_y);
        //printf("step_z = %.3f\n", step_z);
        for (size_t i = 0; i < SamplesSize; i++) {
            for (size_t j = 0; j < SamplesSize; j++) {
                for (size_t k = 0; k < SamplesSize; k++) {
                    double x = bb.xmin() + step_x * (i + 0.5),
                           y = bb.ymin() + step_y * (j + 0.5),
                           z = bb.zmin() + step_z * (k + 0.5);
                    sum[0] += std::abs(fun(x+step_x/2,y,z) - fun(x-step_x/2,y,z));
                    sum[1] += std::abs(fun(x,y+step_y/2,z) - fun(x,y-step_y/2,z));
                    sum[2] += std::abs(fun(x,y,z+step_z/2) - fun(x,y,z-step_z/2));
                }
            }
        }

        //printf("sum_x = %.3f\n", sum[0]);
        //printf("sum_y = %.3f\n", sum[1]);
        //printf("sum_z = %.3f\n", sum[2]);

        double step[] = {step_x, step_y, step_z};
        int dim = 0;
        if (sum[1] > sum[0]) dim = 1;
        if (sum[2] > std::max<double>(sum[0], sum[1])) dim = 2;

        auto fun_ijk = [fun, dim](double x1, double x2, double x3) {
            switch (dim) {
                case 0:
                    return fun(x1, x2, x3);
                case 1:
                    return fun(x2, x3, x1);
                case 2:
                    return fun(x3, x1, x2);
            }
            return 0.;
        };

        double sum_half = 0;
        double best_split;
        for (size_t i = 0; i < SamplesSize; i++) {
            for (size_t j = 0; j < SamplesSize; j++) {
                for (size_t k = 0; k < SamplesSize; k++) {
                    int dim1 = dim,
                        dim2 = (dim+1)%3,
                        dim3 = (dim+2)%3;
                    double x1 = bb.min(dim1) + step[dim1] * i,
                           x2 = bb.min(dim2) + step[dim2] * (j + 0.5),
                           x3 = bb.min(dim3) + step[dim3] * (k + 0.5);
                    sum_half += std::abs(fun_ijk(x1, x2, x3) - fun_ijk(x1+step[dim1], x2, x3));
                }
            }
            if (2 * sum_half > sum[dim]) {
                best_split = bb.min(dim) + step[dim] * (i + 0.5);
                break;
            }
        }

        //printf("sum_half = %.3f\n", sum_half);

        return {
            .pos = best_split,
            .axis = Axis(dim)
        };
    }
};

template<size_t SamplingGridSize>
struct sampling_scalar_error_estimator
{
    static double estimate_error(const Bbox_3 &bb,
                                 const Function3D<double> &func,
                                 double approximation)
    {
        double error = 0.0;
        double dx = (bb.xmax() - bb.xmin()) / (double)SamplingGridSize;
        double dy = (bb.ymax() - bb.ymin()) / (double)SamplingGridSize;
        double dz = (bb.zmax() - bb.zmin()) / (double)SamplingGridSize;

        for (double x = bb.xmin(); x <= bb.xmax() + EPSILON; x += dx) {
            for (double y = bb.ymin(); y <= bb.ymax() + EPSILON; y += dy) {
                for (double z = bb.zmin(); z <= bb.zmax() + EPSILON; z += dz) {
                    error = std::max(std::abs(func(x, y, z) - approximation),
                                     error);
                }
            }
        }

        return error;
    }
};

template<typename ElementT,
         typename BoxSplitter = half_box_splitter<ElementT>,
         typename ErrorEstimator = sampling_scalar_error_estimator<10>>
struct kd_tree
{
    bool is_leaf;
    Bbox_3 bounding_box;

    union kd_tree_data {
        struct {
            ElementT value;
        } leaf;
        struct {
            Axis split_axis;
            double split_pos;
            std::unique_ptr<kd_tree> low;
            std::unique_ptr<kd_tree> high;
        } node;

        // TODO: hack (1/2)
        // unique_ptrs are conditionally freed in ~kd_tree
        ~kd_tree_data() {}
    } data;

    ~kd_tree()
    {
        // TODO: hack (2/2)
        if (is_leaf) {
            data.node.low.~unique_ptr();
            data.node.high.~unique_ptr();
        }
    }

    kd_tree(const kd_tree &src)
    {
        *this = src;
    }
    kd_tree(kd_tree &&src)
    {
        *this = std::move(src);
    }

    kd_tree &operator =(const kd_tree &src)
    {
        ~kd_tree();

        memcpy(this, &src, sizeof(*this));
        if (!src.is_leaf) {
            data.node.low.release();
            data.node.high.release();
            data.node.low = std::move(data.node.low);
            data.node.high = std::move(data.node.high);
        }
        return *this;
    }

    kd_tree &operator =(kd_tree &&src)
    {
        *this = src;

        if (!src.is_leaf) {
            src.data.node.low.release();
            src.data.node.high.release();
        }
        return *this;
    }

    kd_tree(const Bbox_3 &bounding_box,
            const ElementT &value):
        is_leaf(true),
        bounding_box(bounding_box),
        data {
            .leaf = {
                .value = value
            }
        }
    {}

    kd_tree(const Bbox_3 &bounding_box,
            Axis split_axis,
            double split_pos,
            std::unique_ptr<kd_tree> &&low,
            std::unique_ptr<kd_tree> &&high):
        is_leaf(false),
        bounding_box(bounding_box),
        data {
            .node = {
                .split_axis = split_axis,
                .split_pos = split_pos,
                .low = std::move(low),
                .high = std::move(high)
            }
        }
    {}

    struct SubBoxes
    {
        Bbox_3 low;
        Bbox_3 high;
    };

    static SubBoxes split_box(const Bbox_3 &bb,
                              const BoxSplit &split)
    {
        Bbox_3 low;
        Bbox_3 high;

        switch (split.axis) {
            case Axis::X:
                low = Bbox_3(bb.xmin(), bb.ymin(), bb.zmin(),
                             split.pos, bb.ymax(), bb.zmax());
                high = Bbox_3(split.pos, bb.ymin(), bb.zmin(),
                              bb.xmax(), bb.ymax(), bb.zmax());
                break;
            case Axis::Y:
                low = Bbox_3(bb.xmin(), bb.ymin(), bb.zmin(),
                             bb.xmax(), split.pos, bb.zmax());
                high = Bbox_3(bb.xmin(), split.pos, bb.zmin(),
                              bb.xmax(), bb.ymax(), bb.zmax());
                break;
            case Axis::Z:
                low = Bbox_3(bb.xmin(), bb.ymin(), bb.zmin(),
                             bb.xmax(), bb.ymax(), split.pos);
                high = Bbox_3(bb.xmin(), bb.ymin(), split.pos,
                              bb.xmax(), bb.ymax(), bb.zmax());
                break;
        }

        return { low, high };
    }

    static std::unique_ptr<kd_tree<ElementT>>
    build(const Bbox_3 &bb,
          const Function3D<ElementT> &func,
          double max_error)
    {
        Vector_3 center = bb.center();
        double value = func(center.x(), center.y(), center.z());
        double curr_error = ErrorEstimator::estimate_error(bb, func, value);

        if (curr_error < max_error) {
            return std::make_unique<kd_tree<ElementT>>(bb, value);
        } else {
            BoxSplit split = BoxSplitter::split(bb, func);
            SubBoxes sub_bbs = split_box(bb, split);

            //printf("Split at %.3f along %d\n", split.pos, (int) split.axis);

            return std::make_unique<kd_tree<ElementT>>(
                    bb, split.axis, split.pos,
                    std::move(build(sub_bbs.low, func, max_error)),
                    std::move(build(sub_bbs.high, func, max_error)));
        }
    }

    ElementT value_at(double x, double y, double z)
    {
        assert(bounding_box.contains(x, y, z));

        if (is_leaf) {
            return data.leaf.value;
        }

        assert((int)Axis::X <= (int)data.node.split_axis
               && (int)data.node.split_axis <= (int)Axis::Z);

        double pos = (double[]){ x, y, z }[(int)data.node.split_axis];
        return pos < data.node.split_pos ? data.node.low->value_at(x, y, z)
                                         : data.node.high->value_at(x, y, z);
    }
};

