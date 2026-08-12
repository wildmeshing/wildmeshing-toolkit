#pragma once

#include <wmtk/Types.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace wmtk::utils {

/**
 * @brief Options for embed_triangles_in_tets. All checks are off by default.
 */
struct EmbedTrianglesOptions
{
    /// Report input triangles that are collinear, before handing them to the remesher.
    bool check_collinear_input = false;
    /// Check every output tet's orientation in exact arithmetic, after embedding and again
    /// after the vertex compaction.
    bool check_orientation = false;
    /// Report how the provenance-derived "face is on the input surface" answer differs from
    /// the remesher's older colour-derived `facets_on_input`. Diagnostic only -- the surface
    /// always comes from provenance; this just counts the disagreement.
    bool check_surface_provenance = false;
};

/**
 * @brief Which input triangles each output surface face came from.
 *
 * The arrangement answers this per *coplanar group* rather than per input triangle: the
 * remesher first partitions the input into maximal sets of triangles that are transitively
 * edge-adjacent and exactly coplanar, then tracks which output faces tile each set. A flat
 * region tiled by many input triangles is therefore one group, and -- the case to design
 * around -- a group can span triangles from several input surfaces where they meet
 * coplanarly along a shared edge.
 *
 * Optional: pass nullptr (the default) if the boolean `polygon_faces_on_input` is enough,
 * as it is for tetwild, and neither vector is built.
 */
struct EmbedTrianglesProvenance
{
    /// (facet, coplanar group) pairs, sorted, indexing `polygon_faces`. Only facets on the
    /// input surface appear, and a facet where two exactly-coplanar groups meet appears once
    /// per group.
    std::vector<std::array<uint32_t, 2>> face_groups;
    /// Per input triangle (same indexing as `triangle_indices`): its coplanar group, or
    /// UINT32_MAX if it was degenerate and dropped before the arrangement.
    std::vector<uint32_t> triangle_group;
};

/**
 * @brief Conformally insert a triangle soup into a background tet mesh, exactly.
 *
 * Wraps vol_rem::embed_tri_in_poly_mesh (the exact arrangement) and everything that has to
 * happen to its output before a mesh can be built from it:
 *
 *   1. run the arrangement;
 *   2. convert the remesher's bigrational coordinates to Vector3r;
 *   3. decode embedded_facets into triangles (fixed stride of 4: one size prefix + 3 vertex
 *      ids) and recover which are on the input surface, from the remesher's triangle
 *      provenance -- the output faces each coplanar group of input triangles is tiled by;
 *   4. track the surface onto the output tets, using final_tets_parent_faces (which polygon
 *      faces bound each tet);
 *   5. compact away vertices left unreferenced by out_tets and remap every index;
 *   6. reorder the per-tet face flags into WMTK's local face order.
 *
 * Tets come straight from the remesher, so no centroid Steiner points are added and the output
 * has no extra interior vertices. Orientation is passed through: the remesher already emits
 * WMTK-positively oriented tets.
 *
 * Inputs are flat arrays so the caller marshals from whatever it holds -- tetwild has vectors
 * of Vector3d, simwild has Eigen matrices -- and only this middle is shared.
 *
 * @param tri_vrt_coord      input surface vertices, 3 doubles each, xyz-interleaved
 * @param triangle_indices   input surface triangles, 3 vertex ids each
 * @param tet_vrt_coord      background mesh vertices, 3 doubles each
 * @param tet_indices        background mesh tets, 4 vertex ids each
 * @param[out] v_rational    arrangement vertices, exact
 * @param[out] polygon_faces output triangular facets
 * @param[out] polygon_faces_on_input  per facet: does it lie on the input surface
 * @param[out] is_v_on_input per vertex: is it a corner of an on-input facet
 * @param[out] tets_after    output tets
 * @param[out] tet_face_on_input_surface  4 flags per tet, in WMTK local face order
 * @param[out] provenance    optional: which input triangles each surface face came from
 */
void embed_triangles_in_tets(
    const std::vector<double>& tri_vrt_coord,
    const std::vector<uint32_t>& triangle_indices,
    const std::vector<double>& tet_vrt_coord,
    const std::vector<uint32_t>& tet_indices,
    std::vector<Vector3r>& v_rational,
    std::vector<std::array<size_t, 3>>& polygon_faces,
    std::vector<bool>& polygon_faces_on_input,
    std::vector<bool>& is_v_on_input,
    std::vector<std::array<size_t, 4>>& tets_after,
    std::vector<bool>& tet_face_on_input_surface,
    const EmbedTrianglesOptions& opts = {},
    EmbedTrianglesProvenance* provenance = nullptr);

} // namespace wmtk::utils
