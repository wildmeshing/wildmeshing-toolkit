#pragma once

#include "TaggedMesh.hpp"

#include <mshio/mshio.h>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace wmtk::components::polyfem_ops {

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
 * @brief Reduce a multi-tag mesh to the 2-body ("ambient"/"body") mesh polyfem solves on. Mirrors
 * `polyfem_utils._write_polyfem_reduced_msh` up to the write.
 *
 * WMTK's `write_msh_groups` writes one copy of a multi-tagged cell per tag; polyfem reads the
 * copies as distinct elements, double-counting AMIPS and corrupting assembly. Cells are therefore
 * deduped by their vertex SET (the first copy's vertex order is the one kept) and classified by
 * the union of the copies' tag names, into two physical groups: "ambient" with tag 1 and "body"
 * with tag 2, element ids numbered from 1, ambient first.
 *
 * Every original node tag is kept and the nodes are written in ascending tag order, which is what
 * makes polyfem's `in_node_to_node` the identity (input vertex id i <-> gmsh tag i+1) for both the
 * original and the reduced mesh -- the collision artifacts index either one.
 *
 * The result is the file's content as mshio saves it, set up for BINARY msh 4.1: mshio's ASCII
 * writer prints coordinates through a default ostream, which keeps 6 significant digits and would
 * round every coordinate away. Binary stores the doubles themselves, so the reduced mesh carries
 * exactly the coordinates the input had, which is at least as faithful as the gmsh ASCII (%.16g)
 * the Python engine writes -- and it is what makes the file and this content the same mesh.
 */
mshio::MshSpec polyfem_reduced_msh(
    const std::string& input_msh,
    const std::vector<std::string>& ambient_like_tags);

/// Save the reduced mesh to `output_msh`.
void write_polyfem_reduced_msh(const std::string& output_msh, const mshio::MshSpec& reduced);

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
 * @brief Read a .msh's material physical groups. Mirrors `polyfem_utils.get_mesh_info`.
 *
 * The volume is a RUNNING sum (not a pairwise one) over the elements in the order the file lists
 * them -- physical group by physical group, entity by entity, element by element -- each term an
 * absolute determinant over 3 edge vectors / 6 in 3D, or half an absolute cross product in 2D.
 * That sum divides every AMIPS weight in the polyfem JSON, so the order and the per-term
 * arithmetic are part of the contract; see `numpy_det3` for the 3D determinant.
 */
MeshInfo get_mesh_info(const std::string& msh_path);

/// `get_mesh_info` on the mesh a .msh would be saved from. `read_grouped` walks a loaded file and
/// a spec in memory with the same code, so the volumes are summed in the same order either way.
MeshInfo get_mesh_info(const mshio::MshSpec& spec);

} // namespace wmtk::components::polyfem_ops
