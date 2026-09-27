#pragma once

#include "TaggedMesh.hpp"

#include <wmtk/Types.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace wmtk::components::simwild::polyfem_helpers {

/// Which body of the reduced 2-body mesh a cell lands in. The three outcomes of the
/// classification loop in `polyfem_utils._write_polyfem_reduced_msh`.
enum class ReducedBody { ambient, body, skip };

/**
 * @brief Classify one cell by its unioned physical-group NAMES. Mirrors the classification rules
 * of `_write_polyfem_reduced_msh`.
 *
 * A cell carrying "ambient" together with any tag that is not ambient-like is refused: polyfem
 * would have to give one element two materials. A cell whose tags are all ambient-like is ambient
 * (it gets the ambient AMIPS weight and the ambient volume normalization); a cell carrying any
 * other tag is body; a cell with no tags at all is skipped, so it never reaches the reduced mesh.
 *
 * @param ambient_like the `ambient_like_tags` option UNION {"ambient"}, as the Python builds it.
 * @param sorted_vertices the cell's sorted vertex tags, used only in the refusal message.
 */
ReducedBody classify_reduced_cell(
    const TagNames& tags,
    const std::set<std::string>& ambient_like,
    const std::vector<int64_t>& sorted_vertices);

/**
 * @brief The reduced mesh polyfem solves on: the content of `<stem>_polyfem.msh`, as arrays. Built
 * once (`polyfem_reduced_msh`), and then handed to polyfem in memory (PolyfemInProcess.cpp), or
 * written by inputs_only (polyfem_operations.cpp); the material groups of the simulation JSON are
 * read off it (`get_mesh_info`).
 *
 * Two physical groups, "ambient" with tag `ambient_tag` and "body" with tag `body_tag`: the first
 * `n_ambient` rows of `cells` are ambient, the rest body. The file numbers its elements from 1 in
 * this row order, and puts the ambient cells on entity 1 and the body cells on entity 2.
 */
struct ReducedMsh
{
    static constexpr int ambient_tag = 1;
    static constexpr int body_tag = 2;

    int dim = 3; ///< 2 (triangles) or 3 (tetrahedra)
    /// Every node of the input, in node-id order (ascending gmsh tag order for a .msh input),
    /// x y z, with z = 0 in 2D: polyfem's MshReader reads only x and y of a triangle mesh, and the
    /// in-memory mesh takes `leftCols(dim)`. The file numbers them 1..n in this row order, which is
    /// the input's own numbering when its tags are 1..n.
    MatrixXd vertices;
    /// One row per cell: its dim + 1 rows of `vertices`, in the vertex order the input stores.
    MatrixXi cells;
    int64_t n_ambient = 0; ///< the number of ambient cells, which are the first rows of `cells`
};

/**
 * @brief Reduce a multi-tag mesh to the 2-body ("ambient"/"body") mesh polyfem solves on. Mirrors
 * `polyfem_utils._write_polyfem_reduced_msh` up to the write.
 *
 * WMTK's `write_msh_groups` writes one copy of a multi-tagged cell per tag; polyfem reads the
 * copies as distinct elements, double-counting AMIPS and corrupting assembly. The cells are
 * therefore `mesh`'s, which are deduped by their vertex SET (the first copy's vertex order is the
 * one kept), and each is classified by its union of tag names, into the two groups of
 * `ReducedMsh`, ambient first, each in the order `mesh` lists the cells.
 *
 * Every node of `mesh` is kept, in node-id order, which is what makes polyfem's `in_node_to_node`
 * the identity (input vertex id i <-> gmsh tag i+1) for both the original and the reduced mesh --
 * the collision artifacts index either one.
 */
ReducedMsh polyfem_reduced_msh(
    const TaggedMesh& mesh,
    const std::vector<std::string>& ambient_like_tags);

/// What `polyfem_utils.get_mesh_info` returns, in the same order as its 5-tuple.
struct MeshInfo
{
    std::vector<int64_t> tags; ///< the material physical tags, sorted
    int dim = 3; ///< 2 or 3
    std::map<std::string, int64_t> name_to_tag; ///< group name -> tag, empty names dropped
    std::map<int64_t, int64_t> tag_to_count; ///< tag -> element count, deduped per group
    std::map<int64_t, double> tag_to_volume; ///< tag -> rest area/volume in MESH units
};

/**
 * @brief The material physical groups of the reduced mesh. Mirrors `polyfem_utils.get_mesh_info`
 * on the file inputs_only saves.
 *
 * The volume is a RUNNING sum (not a pairwise one) over the elements in the order the file lists
 * them -- physical group by physical group, entity by entity, element by element, which for the
 * reduced mesh is the row order of `cells` -- each term an absolute determinant over 3 edge
 * vectors / 6 in 3D, or half an absolute cross product in 2D. That sum divides every AMIPS weight
 * in the polyfem JSON, so the order and the per-term arithmetic are part of the contract; see
 * `numpy_det3` for the 3D determinant. Both groups are always listed, an empty one with no
 * elements and a zero volume, as the file lists both.
 */
MeshInfo get_mesh_info(const ReducedMsh& reduced);

} // namespace wmtk::components::simwild::polyfem_helpers
