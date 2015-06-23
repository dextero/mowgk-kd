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

typedef CGAL::Cartesian<double> Kernel;
typedef Kernel::Point_3 Vector_3;

template<typename T>
using Function3D = std::function<T(double x, double y, double z)>;

enum class Axis { X = 0, Y = 1, Z = 2 };

constexpr double EPSILON = 1e-5;

/**
 * Defines how the current box should be split.
 */
struct BoxSplit
{
    double pos; /**< Split position on given \p axis. */
    Axis axis;  /**< The axis along which the box should be split. */
};

/**
 * 3D axis-aligned box.
 *
 * Adds some auxiliary methods to the CGAL::Bbox_3 class.
 */
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

    /**
     * Returns the exact middle of the box.
     */
    inline Vector_3 center() const {
        return { (xmin() + xmax()) / 2.0,
                 (ymin() + ymax()) / 2.0,
                 (zmin() + zmax()) / 2.0 };
    }

    /**
     * Returns the box width along given \p axis.
     */
    inline double size(Axis axis) const {
        return max((int)axis) - min((int)axis);
    }

    /**
     * Returns the box width/height/depth as a 3D vector.
     */
    inline Vector_3 size() const {
        return { xmax() - xmin(),
                 ymax() - ymin(),
                 zmax() - zmin() };
    }

    /**
     * Returns the axis along which the box is the largest.
     */
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

    /**
     * Returns the \p axis coordinate of the center of the box.
     */
    inline double half(Axis axis) const {
        return (max((int)axis) + min((int)axis)) / 2.0;
    }

    /**
     * Returns the x coordinate of the center of the box.
     */
    inline double xhalf() const {
        return xmin() + size(Axis::X) / 2.0;
    }
    /**
     * Returns the y coordinate of the center of the box.
     */
    inline double yhalf() const {
        return ymin() + size(Axis::Y) / 2.0;
    }
    /**
     * Returns the z coordinate of the center of the box.
     */
    inline double zhalf() const {
        return zmin() + size(Axis::Z) / 2.0;
    }

    /**
     * Checks if the box contains the { \p x, \p y, \p z } point.
     */
    inline bool contains(double x, double y, double z) const
    {
        return xmin() <= x && x <= xmax()
            && ymin() <= y && y <= ymax()
            && zmin() <= z && z <= zmax();
    }

    /**
     * Checks if the split is valid for the box. Used only for debugging.
     */
    inline bool box_split_valid(const BoxSplit& split) const {
        bool valid = true;

        switch (split.axis) {
        case Axis::X: valid = (xmin() < split.pos && split.pos < xmax()); break;
        case Axis::Y: valid = (ymin() < split.pos && split.pos < ymax()); break;
        case Axis::Z: valid = (zmin() < split.pos && split.pos < zmax()); break;
        }

        if (!valid) {
            fprintf(stderr, "split %f alongside %c is outside box %f, %f, %f - %f, %f %f\n",
                    split.pos, "XYZ"[(int)split.axis],
                    xmin(), ymin(), zmin(), xmax(), ymax(), zmax());
        }

        return valid;
    }
};

/**
 * An example implementation of the BoxSplitter (see \ref kd_tree).
 *
 * Splits the box in the middle along the longest axis.
 */
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

/**
 * An example implementation of the BoxSplitter (see \ref kd_tree).
 *
 * Splits the box according to gradient of the represented function.
 *
 * The current box is split into SamplesSize segments along the axes.
 * Let's introduce split boxes numbering, \f$ S_{i,j,k} \f$
 * where \f$ i,j,k \in [1, \, \text{SamplesSize}] = [1,\,N] \f$.
 * Then we will consider gradient to be the difference of function value
 * between two faces of the box \f$ S_{i,j,k} \f$, i.e.
 * \f{equation}{
 *   \nabla^x_{i,j,k} = f(x_m+\frac{1}{2}d_x, y_m, z_m) - f(x_m-\frac{1}{2}d_x, y_m, z_m)
 * \f}
 * where \f$ d_x \f$ is the size of the box along the x axis
 *   and \f$ x_m, y_m, z_m \f$ are the coordinates of the box's center.
 *
 * For each dimension (x, y and z) the sums of absolute values of
 * gradient are computed as
 * \f{equation}{
 *   \Sigma_x = \sum\limits_{i=1}^N \sum\limits_{j=1}^N \sum\limits_{k=1}^N \left| \nabla^x_{i,j,k} \right|
 *   \text{ .}
 * \f}
 * The axis with the largest sum of gradients is then chosen to be split along.
 *
 * In order to determine the point of split, a value of T is obtained
 * \f{equation}{
 *   T = \min \{ t \mid
 *         \sum\limits_{i=1}^t \sum\limits_{j=1}^N \sum\limits_{k=1}^N
 *           \left| \nabla^x_{i,j,k} \right| > \Sigma_x \}
 * \f}
 * The box is split at the position \f$ x_m \f$ of the box \f$ S_{T,j,k} \f$
 * where \f$ j, k \in [1,\,N] \f$.
 *
 */
template<typename ElementT, size_t SamplesSize>
struct gradient_box_splitter
{
    static_assert(SamplesSize > 0, "number of samples must be nonzero");

    static BoxSplit split(const Bbox_3 &bb,
                          const Function3D<ElementT> &fun)
    {
        std::vector<ElementT> cache;
        cache.reserve(SamplesSize*SamplesSize*SamplesSize);

        double samplesSize = (double) SamplesSize;
        double step_x = (bb.xmax() - bb.xmin()) / samplesSize,
               step_y = (bb.ymax() - bb.ymin()) / samplesSize,
               step_z = (bb.zmax() - bb.zmin()) / samplesSize;

        for (size_t i = 0; i < SamplesSize; i++) {
            for (size_t j = 0; j < SamplesSize; j++) {
                for (size_t k = 0; k < SamplesSize; k++) {
                    double x = bb.xmin() + step_x * (i + 0.5),
                           y = bb.ymin() + step_y * (j + 0.5),
                           z = bb.zmin() + step_z * (k + 0.5);
                    cache.push_back(fun(x,y,z));
                }
            }
        }

        size_t step_i[] = {SamplesSize * SamplesSize,
                           SamplesSize,
                           1};
        auto ind = [&step_i](size_t i, size_t j, size_t k) {
            return step_i[0] * i + step_i[1] * j + step_i[2] * k;
        };

        ElementT sum[] = {0, 0, 0};
        for (size_t i = 0; i < SamplesSize; i++) {
            for (size_t j = 0; j < SamplesSize; j++) {
                for (size_t k = 0; k < SamplesSize; k++) {
                    if (i > 0) sum[0] += std::abs(cache[ind(i,j,k)] - cache[ind(i-1,j,k)]);
                    if (j > 0) sum[1] += std::abs(cache[ind(i,j,k)] - cache[ind(i,j-1,k)]);
                    if (k > 0) sum[2] += std::abs(cache[ind(i,j,k)] - cache[ind(i,j,k-1)]);
                }
            }
        }

        double step[] = {step_x, step_y, step_z};
        int dim = 0;
        if (sum[1] > sum[0]) dim = 1;
        if (sum[2] > sum[dim]) dim = 2;

        auto ind_ijk = [&ind, dim](size_t i, size_t j, size_t k) {
            switch (dim) {
                case 0:
                    return ind(i,j,k);
                case 1:
                    return ind(k,i,j);
                case 2:
                    return ind(j,k,i);
            }
            assert(!"unreachable");
        };

        double sum_half = 0;
        double best_split = bb.min(dim) + step[dim] * SamplesSize / 2;
        for (size_t i = 1; i < SamplesSize; i++) {
            for (size_t j = 0; j < SamplesSize; j++) {
                for (size_t k = 0; k < SamplesSize; k++) {
                    sum_half += std::abs(cache[ind_ijk(i,j,k)] - cache[ind_ijk(i-1,j,k)]);
                }
            }
            if (2 * sum_half > sum[dim]) {
                best_split = bb.min(dim) + step[dim] * i;
                break;
            }
        }

        return {
            .pos = best_split,
            .axis = Axis(dim)
        };
    }
};

/**
 * An example implementation of an ErrorEstimator (see \ref kd_tree).
 *
 * Computes maximum error in specified box with given sampling.
 */
template<size_t SamplingGridSize>
struct sampling_scalar_error_estimator
{
    static_assert(SamplingGridSize > 0, "number of samples must be nonzero");

    /**
     * Returns an error estimation computed as a maximum difference between
     * \p approximation and the \p func values for points from the \p bb,
     * arranged in a regular \p SamplingGridSize x \p SamplingGridSize x
     * \p SamplingGridSize grid.
     *
     * \param[in]\ bb            box to take samples from.
     * \param[in]\ func          function applied on every sample from \p bb.
     * \param[in]\ approximation \p func approximation for the entire \p bb area.
     *
     * \returns Estimated absolute error between \p approximation and \p func
     *          value for points from \p bb.
     */
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

/**
 * Represents the kd-tree.
 *
 * Each element can be a parent or leaf.
 * If it is a leaf then an approximated value is associated with it.
 * If it is a parent it holds information about the way it is split
 * and the two children.
 *
 * \param ElementT    type of elements stored in the tree.
 * \param BoxSplitter functor used to determine optimal split position for
 *                    given area.
 * \param ErrorEstimator functor used to estimate maximum error between the
 *                       actual function value and computed approximation.
 */
template<typename ElementT,
         typename BoxSplitter = half_box_splitter<ElementT>,
         typename ErrorEstimator = sampling_scalar_error_estimator<10>>
class kd_tree final
{
public:
    /**
     * Builds the kd-tree for the specified parameters.
     *
     * The tree is built recursively using \p BoxSplitter and \p ErrorEstimator
     * to guide its creation.
     *
     * \param[in] bb        volume represented by the tree.
     * \param[in] func      function to approximate with the tree.
     * \param[in] max_error largest accepted error between the \p func and
     *                      tree's approximation.
     *
     * \returns constructed kd-tree.
     */
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

            return std::make_unique<kd_tree<ElementT>>(
                    bb, split,
                    std::move(build(sub_bbs.low, func, max_error)),
                    std::move(build(sub_bbs.high, func, max_error)));
        }
    }

    /**
     * Returns the approximated function value at specified point.
     */
    ElementT value_at(double x, double y, double z)
    {
        assert(bounding_box.contains(x, y, z));

        if (is_leaf) {
            return data.leaf.value;
        }

        assert((int)Axis::X <= (int)data.node.split.axis
               && (int)data.node.split.axis <= (int)Axis::Z);

        double pos = (double[]){ x, y, z }[(int)data.node.split.axis];
        return pos < data.node.split.pos ? data.node.low->value_at(x, y, z)
                                         : data.node.high->value_at(x, y, z);
    }

    ~kd_tree()
    {
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

private:
    bool is_leaf;        /**< True for leaf nodes, false otherwise */
    Bbox_3 bounding_box; /**< The volume represented by the tree */

    /**
     * Node-specific data. Implemented as an union to save sizeof(ElementT)
     * bytes per node.
     */
    union kd_tree_data {
        /**
         * Leaf node data.
         */
        struct {
            ElementT value; /**< Approximate function value for the \ref kd_tree#bounding_box volume */
        } leaf;
        /**
         * Parent node data.
         */
        struct {
            BoxSplit split;                /**< Split details. */
            std::unique_ptr<kd_tree> low;  /**< Node below the \p split */
            std::unique_ptr<kd_tree> high; /**< Node above the \p split */
        } node;

        /**
         * HACK: C++ requires non-POD unions to have an explicit destructor
         * defined. unique_ptrs are conditionally freed in ~kd_tree instead of
         * here.
         */
        ~kd_tree_data() {}
    } data;

    /**
     * Constructs a leaf node that approximates the function value on the entire
     * \p bounding_box with \p value.
     *
     * \param[in] bounding_box volume represented by the node.
     * \param[in] value        function value approximation for the \p bounding_box.
     */
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

    /**
     * Constructs a parent node.
     *
     * \param[in] bounding_box volume represented by the node.
     * \param[in] split        volume split details.
     * \param[in] low          node representing the volume below the \p split.
     * \param[in] high         node representing the volume above the \p split.
     */
    kd_tree(const Bbox_3 &bounding_box,
            BoxSplit split,
            std::unique_ptr<kd_tree> &&low,
            std::unique_ptr<kd_tree> &&high):
        is_leaf(false),
        bounding_box(bounding_box),
        data {
            .node = {
                .split = split,
                .low = std::move(low),
                .high = std::move(high)
            }
        }
    {}

    /**
     * Auxiliary struct to allow returning multiple values from the
     * \ref kd_tree#split_box.
     */
    struct SubBoxes
    {
        Bbox_3 low;
        Bbox_3 high;
    };

    /**
     * Splits the \p bb into two boxes according to \p split.
     */
    static SubBoxes split_box(const Bbox_3 &bb,
                              const BoxSplit &split)
    {
        assert(bb.box_split_valid(split));

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
};

