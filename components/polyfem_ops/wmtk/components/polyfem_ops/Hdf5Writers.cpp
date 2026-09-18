#include "Hdf5Writers.hpp"

#include "ConstraintMatrices.hpp"
#include "NumpyCompat.hpp"

#include <wmtk/utils/Logger.hpp>

// h5pp comes in through polyfem::polyfem (polyfem -> paraviewo -> h5pp, with HDF5 on), the same
// way polyfem itself reaches it (`#include <h5pp/h5pp.h>` in its sources), so the component's
// CMakeLists needs no extra link target for it.
#include <h5pp/h5pp.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace wmtk::components::polyfem_ops {

namespace {

/// int32 copy of the node ids. The Python builds `node_ids` as an np.int32 array in
/// make_interface_constraint (and the pin writer casts explicitly), and h5pp reads local2global
/// back as vector<int>, so int32 is what has to be on disk.
std::vector<int32_t> as_int32(const std::vector<int64_t>& ids)
{
    std::vector<int32_t> out(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
        out[i] = static_cast<int32_t>(ids[i]);
    }
    return out;
}

/// Write the four A_triplets datasets plus local2global and b, the layout every constraint file
/// shares (polyfem SolveData.cpp).
void write_constraint_file(
    const std::string& path,
    const std::vector<int32_t>& local2global,
    const Triplets& a,
    const std::array<int64_t, 2>& shape,
    const std::vector<double>& b,
    int64_t b_rows,
    int64_t b_cols)
{
    h5pp::File file(path, h5pp::FileAccess::REPLACE);
    file.writeDataset(local2global, "local2global");
    file.writeDataset(a.rows, "A_triplets/rows");
    file.writeDataset(a.cols, "A_triplets/cols");
    file.writeDataset(a.values, "A_triplets/values");
    file.writeDataset(std::vector<int64_t>{shape[0], shape[1]}, "A_triplets/shape");
    file.writeDataset(b, "b", {b_rows, b_cols});
}

/// One row of the CSR form of a COO matrix: the column indices in ascending order and their
/// values. Mirrors what `scipy.sparse.coo_matrix(...).tocsr()` produces -- scipy's coo_tocsr
/// buckets the entries by row keeping their COO order, and the sum_duplicates() that follows
/// sorts each row by column and adds equal columns together. The product below then runs over a
/// row in exactly this order, which is what fixes the rounding of `b`.
struct CsrRows
{
    std::vector<std::vector<std::pair<int32_t, double>>> rows;
};

CsrRows to_csr(const Triplets& t, int64_t n)
{
    CsrRows csr;
    csr.rows.resize(static_cast<size_t>(n));
    for (size_t k = 0; k < t.values.size(); ++k) {
        csr.rows[static_cast<size_t>(t.rows[k])].emplace_back(t.cols[k], t.values[k]);
    }
    for (auto& row : csr.rows) {
        std::stable_sort(row.begin(), row.end(), [](const auto& x, const auto& y) {
            return x.first < y.first;
        });
        // Duplicate columns are summed in the sorted order, as csr_sum_duplicates does. The
        // matrices written here have unique (row, col) pairs, so this only ever copies.
        std::vector<std::pair<int32_t, double>> merged;
        for (const auto& [col, value] : row) {
            if (!merged.empty() && merged.back().first == col) {
                merged.back().second += value;
            } else {
                merged.emplace_back(col, value);
            }
        }
        row = std::move(merged);
    }
    return csr;
}

} // namespace

void write_fitting_constraint_hdf5(
    const std::string& path,
    const std::vector<int64_t>& node_ids,
    int dim,
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    bool graph,
    bool normalize,
    const std::vector<std::array<int64_t, 3>>& interface_faces)
{
    const int64_t n = static_cast<int64_t>(node_ids.size());
    Triplets a = get_mass_matrix(coords, interface_edges, node_ids, graph, interface_faces);

    if (normalize) {
        // np.sum over the mass values, then a division of each sqrt by the sqrt of that total:
        // two separate square roots, not one of the ratio.
        const double l_total = pairwise_sum(a.values);
        const double denom = std::sqrt(l_total);
        for (double& v : a.values) {
            v = std::sqrt(v) / denom;
        }
        logger().info("  fit norm   : L_total = {:.6g} (mesh units)", l_total);
    } else {
        for (double& v : a.values) {
            v = std::sqrt(v);
        }
    }

    write_constraint_file(
        path,
        as_int32(node_ids),
        a,
        {n, n},
        std::vector<double>(static_cast<size_t>(n * dim), 0.0),
        n,
        dim);
}

void write_laplacian_constraint_hdf5(
    const std::string& path,
    const std::vector<int64_t>& node_ids,
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    bool graph,
    double scale,
    bool normalize,
    const std::vector<std::array<int64_t, 3>>& interface_faces,
    bool smooth_positions)
{
    const int64_t n = static_cast<int64_t>(node_ids.size());
    const int64_t dim = coords.cols();

    // Zero-row-sum check on S, where the property is constructed exactly. L = M^-1 S also has
    // zero row sums in exact arithmetic, but a wide mass range (small triangles -> big M_inv)
    // amplifies float noise in S's row sums past any fixed absolute threshold on L.
    {
        const Triplets s =
            get_stiffness_matrix(coords, interface_edges, node_ids, graph, interface_faces);
        const CsrRows csr = to_csr(s, n);
        double max_abs_row_sum = 0.0;
        for (const auto& row : csr.rows) {
            double sum = 0.0;
            for (const auto& [col, value] : row) {
                sum += value;
            }
            max_abs_row_sum = std::max(max_abs_row_sum, std::abs(sum));
        }
        double s_scale = 1.0;
        for (const double v : s.values) {
            s_scale = std::max(s_scale, std::abs(v));
        }
        if (!(max_abs_row_sum < 1e-10 * s_scale)) {
            log_and_throw_error(
                "S should have zero row sums (max |row_sum| = {:.3e}, scale = {:.3e})",
                max_abs_row_sum,
                s_scale);
        }
    }

    Triplets a = get_laplacian_matrix(coords, interface_edges, node_ids, graph, interface_faces);
    if (normalize) {
        std::vector<double> squares(a.values.size());
        for (size_t i = 0; i < a.values.size(); ++i) {
            squares[i] = a.values[i] * a.values[i];
        }
        const double l_frob = std::sqrt(pairwise_sum(squares));
        for (double& v : a.values) {
            v = v / l_frob;
        }
        logger().info("  lap norm   : ||L||_F = {:.6g} (mesh units)", l_frob);
    }

    std::vector<double> b(static_cast<size_t>(n * dim), 0.0);
    if (smooth_positions) {
        // b = -L @ X with X = scale * rest coordinates. Python's unary minus binds tighter than
        // `@`, so scipy negates the stored values first and then accumulates them into a
        // zero-initialized result, one row at a time in CSR order.
        //
        // std::fma, not a multiply and an add: scipy's csr_matvecs accumulates with
        // `y[k] += a * x[k]`, a single statement, which clang contracts into a FUSED
        // multiply-add on arm64 -- one rounding instead of two. Measured on the jagged2d
        // interface, whose rows cancel to ~1e-20: separate rounding gives exact zeros where
        // scipy gives 1e-20 residuals, and reproducing the fusion makes all 124 entries of `b`
        // bit-identical.
        std::vector<double> x(static_cast<size_t>(n * dim));
        for (int64_t i = 0; i < n; ++i) {
            for (int64_t k = 0; k < dim; ++k) {
                x[static_cast<size_t>(i * dim + k)] = scale * coords(node_ids[static_cast<size_t>(i)], k);
            }
        }
        const CsrRows csr = to_csr(a, n);
        for (int64_t i = 0; i < n; ++i) {
            for (const auto& [col, value] : csr.rows[static_cast<size_t>(i)]) {
                const double negated = -value;
                for (int64_t k = 0; k < dim; ++k) {
                    double& acc = b[static_cast<size_t>(i * dim + k)];
                    acc = std::fma(negated, x[static_cast<size_t>(col * dim + k)], acc);
                }
            }
        }
    }

    write_constraint_file(path, as_int32(node_ids), a, {n, n}, b, n, dim);
}

void write_pin_constraint_hdf5(
    const std::string& path,
    const std::vector<int64_t>& node_ids,
    int dim,
    const std::optional<std::vector<int>>& axes)
{
    const int64_t n = static_cast<int64_t>(node_ids.size());
    Triplets a;
    std::string what;
    int64_t b_rows = n;
    int64_t b_cols = dim;
    std::array<int64_t, 2> shape{n, n};

    if (!axes.has_value()) {
        a.rows.resize(static_cast<size_t>(n));
        a.cols.resize(static_cast<size_t>(n));
        a.values.assign(static_cast<size_t>(n), 1.0);
        for (int64_t i = 0; i < n; ++i) {
            a.rows[static_cast<size_t>(i)] = static_cast<int32_t>(i);
            a.cols[static_cast<size_t>(i)] = static_cast<int32_t>(i);
        }
        what = "all axes";
    } else {
        const std::vector<int>& ax = *axes;
        if (ax.empty() || std::any_of(ax.begin(), ax.end(), [dim](int a_) {
                return a_ < 0 || a_ >= dim;
            })) {
            log_and_throw_error("axes out of range for a {}D mesh", dim);
        }
        const int64_t m = n * static_cast<int64_t>(ax.size());
        a.rows.resize(static_cast<size_t>(m));
        a.cols.reserve(static_cast<size_t>(m));
        a.values.assign(static_cast<size_t>(m), 1.0);
        for (int64_t i = 0; i < m; ++i) {
            a.rows[static_cast<size_t>(i)] = static_cast<int32_t>(i);
        }
        for (int64_t i = 0; i < n; ++i) {
            for (const int axis : ax) {
                a.cols.push_back(static_cast<int32_t>(i * dim + axis));
            }
        }
        shape = {m, n * dim};
        b_rows = m;
        b_cols = 1;
        for (const int axis : ax) {
            what += "xyz"[axis];
        }
    }

    write_constraint_file(
        path,
        as_int32(node_ids),
        a,
        shape,
        std::vector<double>(static_cast<size_t>(b_rows * b_cols), 0.0),
        b_rows,
        b_cols);
    logger().info("  pinned     : {}  ({} nodes, {})", path, n, what);
}

void write_linear_map_hdf5(
    const std::string& path,
    const std::vector<int64_t>& node_ids,
    int64_t total_n_nodes)
{
    const int64_t n = static_cast<int64_t>(node_ids.size());
    std::vector<int32_t> rows(static_cast<size_t>(n));
    for (int64_t i = 0; i < n; ++i) {
        rows[static_cast<size_t>(i)] = static_cast<int32_t>(i);
    }
    const std::vector<int32_t> cols = as_int32(node_ids);
    const std::vector<double> values(static_cast<size_t>(n), 1.0);
    const std::vector<int64_t> shape{n, total_n_nodes};

    h5pp::File file(path, h5pp::FileAccess::REPLACE);
    file.writeDataset(rows, "weight_triplets/rows");
    file.writeDataset(cols, "weight_triplets/cols");
    file.writeDataset(values, "weight_triplets/values");
    // An ATTRIBUTE on the group, not a dataset -- polyfem CollisionProxy.cpp reads it there.
    file.writeAttribute(shape, "weight_triplets", "shape");

    logger().info("  linear map : {}  (shape [{}, {}])", path, n, total_n_nodes);
}

} // namespace wmtk::components::polyfem_ops
