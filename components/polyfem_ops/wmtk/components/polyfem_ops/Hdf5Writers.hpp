#pragma once

#include "ConstraintMatrices.hpp"

#include <wmtk/Types.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace wmtk::components::polyfem_ops {

/**
 * @brief The datasets of one constraint HDF5, the layout every constraint file shares (polyfem
 * SolveData.cpp): local2global (int32), A_triplets/{rows,cols} (int32), A_triplets/values
 * (float64), A_triplets/shape (int64) and b (float64, `b_rows` x `b_cols`).
 *
 * Built once and then either written (`write_constraint_hdf5`) or handed to polyfem in memory
 * (PolyfemInProcess.cpp), so the file and the in-memory constraint are the same arrays.
 */
struct ConstraintHdf5
{
    std::vector<int32_t> local2global;
    Triplets a;
    std::array<int64_t, 2> shape{};
    std::vector<double> b; ///< row-major, which is the order the file stores it in
    int64_t b_rows = 0;
    int64_t b_cols = 0;
};

/**
 * @brief The fitting constraint. Mirrors `constraints.write_fitting_constraint_hdf5` up to the
 * write.
 *
 * A = sqrt(M), so (1/2)||A u||^2 is a proper interface L2 norm; `normalize` also divides A by
 * sqrt(L_total) so a uniform displacement u_ref costs w * u_ref^2. A is [n, n] and b is n x dim
 * zeros.
 */
ConstraintHdf5 fitting_constraint(
    const std::vector<int64_t>& node_ids,
    int dim,
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    bool graph,
    bool normalize,
    const std::vector<std::array<int64_t, 3>>& interface_faces);

/**
 * @brief The Laplacian smoothness constraint with A = L. Mirrors
 * `constraints.write_laplacian_constraint_hdf5` up to the write.
 *
 * Default b = 0 enforces harmonic displacements (L u = 0); `smooth_positions` instead sets
 * b = -L (scale * rest_coords) so absolute positions are harmonic (scale = solver units per mesh
 * unit). `normalize` divides A by ||L||_F. The zero-row-sum check is made on the stiffness matrix
 * S, where the property is constructed exactly, and not on L, whose row sums float noise gets
 * amplified by a wide mass range -- the Python asserts in the same place for the same reason.
 */
ConstraintHdf5 laplacian_constraint(
    const std::vector<int64_t>& node_ids,
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    bool graph,
    double scale,
    bool normalize,
    const std::vector<std::array<int64_t, 3>>& interface_faces,
    bool smooth_positions);

/**
 * @brief A pin constraint (A u = 0 on node_ids). Mirrors `constraints.write_pin_constraint_hdf5`
 * up to the write.
 *
 * An empty `axes` optional pins every component. Otherwise `axes` is a subset of the component
 * indices (0=x, 1=y, 2=z) and only those are held, leaving the node free to slide along the rest.
 * The two cases use polyfem's two scatter_matrix modes: with b of `dim` columns each triplet is
 * replicated across all components (node-wise); with b of ONE column the triplet's column index
 * is a flattened degree of freedom (node * dim + component), which is what lets a single axis be
 * pinned.
 */
ConstraintHdf5 pin_constraint(
    const std::vector<int64_t>& node_ids,
    int dim,
    const std::optional<std::vector<int>>& axes);

/// Write a constraint file, dataset for dataset what the Python's three constraint writers write.
void write_constraint_hdf5(const std::string& path, const ConstraintHdf5& constraint);

/// The identity displacement map proxy_vert[i] <- fe_node[node_ids[i]]: the weight_triplets group
/// and its `shape` [n_proxy, total_n_nodes], which the file stores as an ATTRIBUTE of the group
/// (not a dataset), per polyfem CollisionProxy.cpp.
struct LinearMapHdf5
{
    std::vector<int32_t> rows;
    std::vector<int32_t> cols;
    std::vector<double> values;
    std::array<int64_t, 2> shape{};
};

/// Mirrors `constraints.write_linear_map_hdf5` up to the write.
LinearMapHdf5 linear_map(const std::vector<int64_t>& node_ids, int64_t total_n_nodes);

/// Write the linear map file, dataset and attribute for what the Python writes.
void write_linear_map_hdf5(const std::string& path, const LinearMapHdf5& map);

} // namespace wmtk::components::polyfem_ops
