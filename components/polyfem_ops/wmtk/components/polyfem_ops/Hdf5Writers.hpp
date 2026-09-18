#pragma once

#include <wmtk/Types.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace wmtk::components::polyfem_ops {

/**
 * @brief Write the fitting constraint HDF5. Mirrors `constraints.write_fitting_constraint_hdf5`.
 *
 * A = sqrt(M), so (1/2)||A u||^2 is a proper interface L2 norm; `normalize` also divides A by
 * sqrt(L_total) so a uniform displacement u_ref costs w * u_ref^2. Datasets: local2global (int32),
 * A_triplets/{rows,cols} (int32), A_triplets/values (float64), A_triplets/shape (int64, [n, n]),
 * b (float64, n x dim, zeros).
 */
void write_fitting_constraint_hdf5(
    const std::string& path,
    const std::vector<int64_t>& node_ids,
    int dim,
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    bool graph,
    bool normalize,
    const std::vector<std::array<int64_t, 3>>& interface_faces);

/**
 * @brief Write the Laplacian smoothness constraint HDF5 with A = L. Mirrors
 * `constraints.write_laplacian_constraint_hdf5`.
 *
 * Default b = 0 enforces harmonic displacements (L u = 0); `smooth_positions` instead sets
 * b = -L (scale * rest_coords) so absolute positions are harmonic (scale = solver units per mesh
 * unit). `normalize` divides A by ||L||_F. The zero-row-sum check is made on the stiffness matrix
 * S, where the property is constructed exactly, and not on L, whose row sums float noise gets
 * amplified by a wide mass range -- the Python asserts in the same place for the same reason.
 */
void write_laplacian_constraint_hdf5(
    const std::string& path,
    const std::vector<int64_t>& node_ids,
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    bool graph,
    double scale,
    bool normalize,
    const std::vector<std::array<int64_t, 3>>& interface_faces,
    bool smooth_positions);

/**
 * @brief Write a pin constraint (A u = 0 on node_ids). Mirrors
 * `constraints.write_pin_constraint_hdf5`.
 *
 * An empty `axes` optional pins every component. Otherwise `axes` is a subset of the component
 * indices (0=x, 1=y, 2=z) and only those are held, leaving the node free to slide along the rest.
 * The two cases use polyfem's two scatter_matrix modes: with b of `dim` columns each triplet is
 * replicated across all components (node-wise); with b of ONE column the triplet's column index
 * is a flattened degree of freedom (node * dim + component), which is what lets a single axis be
 * pinned.
 */
void write_pin_constraint_hdf5(
    const std::string& path,
    const std::vector<int64_t>& node_ids,
    int dim,
    const std::optional<std::vector<int>>& axes);

/// Write the identity displacement map proxy_vert[i] <- fe_node[node_ids[i]] as a weight_triplets
/// HDF5 group with a `shape` ATTRIBUTE [n_proxy, total_n_nodes] (not a dataset), per polyfem
/// CollisionProxy.cpp. Mirrors `constraints.write_linear_map_hdf5`.
void write_linear_map_hdf5(
    const std::string& path,
    const std::vector<int64_t>& node_ids,
    int64_t total_n_nodes);

} // namespace wmtk::components::polyfem_ops
