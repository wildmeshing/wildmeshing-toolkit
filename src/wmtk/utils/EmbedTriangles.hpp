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
 *      ids) and recover which are on the input surface;
 *   4. track the surface onto the output tets, using the remesher's own metadata --
 *      final_tets_parent (which polyhedral cell each tet came from), final_tets_parent_faces
 *      (which polygon faces bound it) and cells_with_faces_on_input;
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
    const EmbedTrianglesOptions& opts = {});

} // namespace wmtk::utils
