#pragma once

#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/expression_parser/Expression.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace wmtk::components::polyfem_ops {

/// The set of physical-group NAMES carried by one cell. The Python side uses a frozenset of
/// strings for the same thing (mesh_core.TaggedMesh.prim_tags).
using TagNames = std::set<std::string>;

/**
 * @brief One .msh, read the way every mirrored Python function that opens one reads it: gmsh's
 * node list in file order and the cells of every physical group of the mesh's own dimension,
 * group by group (ascending tag, as `gmsh.model.getPhysicalGroups` hands them back), entity by
 * entity (ascending tag, as `gmsh.model.getEntitiesForPhysicalGroup` hands them back), element by
 * element.
 *
 * `mesh_core.TaggedMesh.__init__`, `polyfem_utils.get_mesh_info` and
 * `polyfem_utils._write_polyfem_reduced_msh` each open the file and walk it themselves, and the
 * three walk it in exactly this order; what differs between them is only what they record per
 * element, so the traversal is done once here and each mirror keeps its own recording step.
 */
struct GroupedMsh
{
    int dim = 3;
    std::vector<int64_t> node_tags; ///< file order
    std::vector<std::array<double, 3>> node_coords; ///< file order, always three components
    /// One entry per (group, entity, element) in traversal order: the group's name and tag, the
    /// element's own tag and its vertex tags in the order the file stores them.
    struct Item
    {
        std::string group_name;
        int64_t group_tag = 0;
        int64_t element_tag = 0;
        std::vector<int64_t> nodes;
    };
    std::vector<Item> items;
    /// group tag -> name, in ascending tag order; a group with no name keeps the empty string,
    /// exactly as `gmsh.model.getPhysicalName` returns it.
    std::vector<std::pair<int64_t, std::string>> groups;
};

GroupedMsh read_grouped(const std::string& msh_path);

/// gmsh node tag -> 0-based node id, the rule `polyfem_utils.read_msh_nodes` and
/// `mesh_core.TaggedMesh` both apply: `tag - 1` when the largest tag equals the node count (the
/// contiguous case, which is what polyfem's MshReader assumes, so the constraint columns
/// reference the right FE nodes), otherwise the rank in sorted tag order.
std::map<int64_t, int64_t> node_tag_to_index(const std::vector<int64_t>& tags);

/**
 * @brief A parsed tag expression, evaluable over a cell's set of group names.
 *
 * Mirrors `mesh_core.parse_expression`, which itself delegates to this same C++ parser through
 * the `wildmeshing.Expression` binding (app/pywildmeshing/pywildmeshing.cpp): one grammar, one
 * implementation. Only the identifier lexer and the name-to-id assignment are mirrored here, and
 * they are mirrored exactly, because they decide which names an expression is allowed to see.
 */
class CompiledExpression
{
public:
    /// Identifiers appearing in an expression (lexing only, no grammar). Mirrors
    /// `mesh_core._expr_atoms`: a run of alphanumerics, '_' or '-'. Python's str.isalnum() is
    /// Unicode-aware and this is ASCII-only; physical-group names in a .msh are ASCII.
    static std::set<std::string> atoms(const std::string& expr);

    /// Parse `expr`, declaring exactly its own atoms as the known names, as
    /// `mesh_core.parse_expression` does. Throws on a syntax error.
    explicit CompiledExpression(const std::string& expr);

    /// Mirrors PyExpression::eval: names outside the expression's own atoms still enter the tag
    /// set under fresh ids, so they can never satisfy a name atom but `_` still sees them.
    bool eval(const TagNames& tags) const;

    const std::set<std::string>& names() const { return m_names; }

private:
    std::map<std::string, int64_t> m_name_to_id;
    std::set<std::string> m_names;
    simwild::expression_parser::ExpressionPtr m_expr;
};

/// One normalized selection: the boundary of `region`, kept where the outside cell satisfies
/// `filter`. Mirrors the (region, filter, id) triple `mesh_core.normalize_selection` returns.
struct Selection
{
    std::string region;
    std::optional<std::string> filter;
    std::optional<int64_t> id;
};

/// Mirrors `mesh_core.normalize_selection`: a bare string is a region; an object takes
/// region/filter/id and nothing else.
Selection normalize_selection(const nlohmann::json& spec);

/**
 * @brief Mirrors `mesh_core.assign_selection_ids`.
 *
 * Identical (region, filter) specs collapse to ONE selection with one id (conflicting explicit
 * ids on the same selection throw; distinct selections may share an explicit id to form one
 * body). Auto ids are assigned sequentially and never collide with explicit ones.
 *
 * @param[out] unique selections in first-appearance order, each with its id resolved
 * @param[out] ids_per_input the id of each input spec, in input order
 */
void assign_selection_ids(
    const nlohmann::json& selections,
    std::vector<Selection>& unique,
    std::vector<int64_t>& ids_per_input,
    bool require_ids = false);

/**
 * @brief A physical-groups .msh with WMTK's duplicate cells merged. Mirrors
 * `mesh_core.TaggedMesh`.
 *
 * WMTK's `write_msh_groups` writes one copy of a multi-tagged cell per tag in its tag set; the
 * copies are merged back into one cell carrying the union of the names, keyed by the cell's node
 * set, exactly as the Python `canonical` map does.
 *
 * Read with mshio rather than with simwild's `read_image_msh`: that reader concatenates and
 * re-indexes the vertices of each physical group, which loses the gmsh node tags. The node ids
 * here have to be polyfem's (tag - 1 on a contiguous mesh) or the constraint columns would point
 * at the wrong FE nodes.
 *
 * Every container that the outputs' ORDER depends on is a vector in first-appearance order,
 * because the Python dicts it mirrors are insertion-ordered and that order reaches the OBJ (which
 * face is written first, and which of its three vertices is written first).
 */
class TaggedMesh
{
public:
    explicit TaggedMesh(const std::string& msh_path);

    int64_t total_n_nodes = 0;
    int mesh_dim = 3;
    /// gmsh node tag -> 0-based node id. polyfem's MshReader uses tag - 1 on a contiguous mesh.
    std::map<int64_t, int64_t> node_tag_to_idx;
    /// (total_n_nodes, mesh_dim), indexed by node id, in mesh units.
    MatrixXd coords;
    /// physical group name -> physical tag, for the groups of dimension mesh_dim.
    std::map<std::string, int64_t> names;

    /// Cells, in first-appearance order. prim_nodes[p] is the 0-based vertex tuple of the first
    /// copy seen; prim_tags[p] is the union of the names of every copy.
    std::vector<std::vector<int64_t>> prim_nodes;
    std::vector<TagNames> prim_tags;

    /// Faces (triangles in 3D, edges in 2D), in first-appearance order. face_repr[f] is the
    /// ordered vertex tuple of the first cell that touched the face; face_to_prims[f] lists the
    /// incident cells in the order they were seen.
    std::vector<std::vector<int64_t>> face_repr;
    std::vector<std::vector<int64_t>> face_to_prims;

    /// Mean of the cell's vertex coordinates. Mirrors `TaggedMesh.centroid`.
    VectorXd centroid(int64_t prim) const;

    size_t num_faces() const { return face_repr.size(); }
};

/**
 * @brief Sorted 0-indexed node ids of every cell whose tag set satisfies any of the expressions.
 * Mirrors `mesh_core.select_region_nodes`.
 *
 * Throws if an expression selects no cells (almost always a typo'd tag). Unlike a selection's
 * region/filter, these expressions are compiled without the unknown-name and '_' checks -- the
 * Python calls `parse_expression` directly here, and an unknown name simply matches nothing and
 * is then reported as the empty selection.
 */
std::vector<int64_t> select_region_nodes(
    const TaggedMesh& mesh,
    const std::vector<std::string>& exprs);

/// One selected oriented boundary face. Mirrors a record of `mesh_core.select_boundary_faces`.
struct BoundaryFaceRecord
{
    std::vector<int64_t> face; ///< the face's vertex tuple, as stored in face_repr
    int64_t a_prim = -1; ///< the inside cell
    int64_t b_prim = -1; ///< the outside cell
    std::vector<int64_t> ids; ///< sorted selection ids that picked this face
};

/**
 * @brief Mirrors `mesh_core.select_boundary_faces`.
 *
 * A face is selected iff its inside cell satisfies `region`, the outside does not, and -- when
 * given -- the outside satisfies `filter`. Interior faces only; the domain boundary is never
 * selected. Orientation is left to the caller: outward from the region (A -> B).
 */
std::vector<BoundaryFaceRecord> select_boundary_faces(
    const TaggedMesh& mesh,
    const std::vector<Selection>& selections);

} // namespace wmtk::components::polyfem_ops
