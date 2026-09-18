#pragma once

#include <wmtk/Types.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace wmtk::components::polyfem_ops {

/// A COO matrix over the interface patch, indexed LOCALLY (local index = position in node_ids).
/// The order of the entries is part of the HDF5 contract -- polyfem re-assembles them in the
/// order given -- so every producer below states which order it reproduces and why.
struct Triplets
{
    std::vector<int32_t> rows;
    std::vector<int32_t> cols;
    std::vector<double> values;

    size_t size() const { return values.size(); }
};

/**
 * @brief Lumped mass matrix over the interface as diagonal COO triplets. Mirrors
 * `constraints.get_mass_matrix`.
 *
 * 3D (non-empty `interface_faces`): igl's barycentric mass over the local interface mesh, the
 * same call the Python makes (`igl.massmatrix(V, F, igl.MASSMATRIX_TYPE_BARYCENTRIC)`); 2D: half
 * an edge length on each endpoint; graph: the identity. `graph` is tested first, exactly as in
 * the Python, so a graph run in 3D also takes the edge-length path.
 */
Triplets get_mass_matrix(
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    const std::vector<int64_t>& node_ids,
    bool graph,
    const std::vector<std::array<int64_t, 3>>& interface_faces);

/**
 * @brief Stiffness matrix S over the interface as COO triplets (zero row sums, off-diagonal <= 0,
 * diagonal >= 0). Mirrors `constraints.get_stiffness_matrix`.
 *
 * 3D: the negated `igl.cotmatrix`, whose triplets the Python takes from the CSC matrix igl
 * returns, so the entry order is COLUMN-major with ascending row inside a column; 2D and graph: a
 * per-edge accumulation into a Python `defaultdict`, whose INSERTION order is the entry order --
 * per edge (la,lb), (lb,la), (la,la), (lb,lb), each entry appearing at the position of its first
 * touch.
 */
Triplets get_stiffness_matrix(
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    const std::vector<int64_t>& node_ids,
    bool graph,
    const std::vector<std::array<int64_t, 3>>& interface_faces);

/// Laplacian L = M^-1 S as COO triplets: the stiffness triplets scaled row-wise by the inverse
/// lumped mass diagonal, in the stiffness order. Mirrors `constraints.get_laplacian_matrix`.
Triplets get_laplacian_matrix(
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    const std::vector<int64_t>& node_ids,
    bool graph,
    const std::vector<std::array<int64_t, 3>>& interface_faces);

/// `'z' | 'xy' | [2] | null -> sorted component indices` (empty optional = every component).
/// Mirrors `constraints.parse_axes`. Only the string form is reachable through the spec
/// (/protected_regions/*/axes is typed "string"); the integer-list form is mirrored because the
/// Python accepts it from a direct engine call.
std::optional<std::vector<int>> parse_axes(const nlohmann::json& spec, int dim);

} // namespace wmtk::components::polyfem_ops
