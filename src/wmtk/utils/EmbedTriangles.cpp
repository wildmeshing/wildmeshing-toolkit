#include <wmtk/utils/EmbedTriangles.hpp>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/predicates.hpp>

// MatrixBase::cross is declared by Eigen/Core but only DEFINED in Eigen/Geometry, so without
// this the two normal computations below compile and then fail to link.
#include <Eigen/Geometry>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <VolumeRemesher/embed.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <set>

namespace wmtk::utils {

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
    const EmbedTrianglesOptions& opts)
{
    // Remesher outputs. The tet-based ones are what this consumes: out_tets (the
    // remesher's tetrahedra), final_tets_parent (parent polyhedral cell of each
    // tet), cells_with_faces_on_input (per-cell flag) and final_tets_parent_faces
    // (the parent faces bounding each tet). embedded_cells is not decoded.
    std::vector<vol_rem::bigrational> embedded_vertices;
    std::vector<uint32_t> embedded_facets;
    std::vector<uint32_t> embedded_cells;
    std::vector<uint32_t> embedded_facets_on_input;

    std::vector<std::array<uint32_t, 4>> out_tets;
    std::vector<uint32_t> final_tets_parent;
    std::vector<bool> cells_with_faces_on_input;
    std::vector<std::vector<uint32_t>> final_tets_parent_faces;

    // Warn about degenerate (collinear) input triangles before embedding.
    if (opts.check_collinear_input) {
        logger().warn("Check collinearity before embedding");
        for (int i = 0; i < triangle_indices.size(); i += 3) {
            int id0 = triangle_indices[i + 0];
            int id1 = triangle_indices[i + 1];
            int id2 = triangle_indices[i + 2];
            Vector3d v0(
                tri_vrt_coord[3 * id0 + 0],
                tri_vrt_coord[3 * id0 + 1],
                tri_vrt_coord[3 * id0 + 2]);
            Vector3d v1(
                tri_vrt_coord[3 * id1 + 0],
                tri_vrt_coord[3 * id1 + 1],
                tri_vrt_coord[3 * id1 + 2]);
            Vector3d v2(
                tri_vrt_coord[3 * id2 + 0],
                tri_vrt_coord[3 * id2 + 1],
                tri_vrt_coord[3 * id2 + 2]);

            if (utils::predicates::is_degenerate(v0, v1, v2)) {
                logger().error(
                    "Face ({}, {}, {}) is collinear!",
                    v0.transpose(),
                    v1.transpose(),
                    v2.transpose());
            }
        }
        logger().warn("Check done");
    }

    // Step 3: run the exact arrangement.
    // volumeremesher embed
    std::vector<double> vr_edge_coords, vr_point_coords;
    std::vector<uint32_t> vr_edge_indexes;
    std::vector<std::vector<std::array<uint32_t, 4>>> vr_tri_provenance;
    std::vector<std::vector<std::array<uint32_t, 3>>> vr_edge_provenance;
    std::vector<std::array<uint32_t, 2>> vr_point_provenance;
    vol_rem::embed_tri_in_poly_mesh(
        tri_vrt_coord,
        triangle_indices,
        tet_vrt_coord,
        tet_indices,
        embedded_vertices,
        embedded_facets,
        embedded_cells,
        out_tets,
        final_tets_parent,
        embedded_facets_on_input,
        cells_with_faces_on_input,
        final_tets_parent_faces,
        vr_edge_coords,
        vr_edge_indexes,
        vr_point_coords,
        vr_tri_provenance,
        vr_edge_provenance,
        vr_point_provenance,
        true);

    // Step 4a: copy the arrangement vertices to exact rational Vector3r. No
    // compaction yet -- unused vertices are pruned near the end.
    for (int i = 0; i < embedded_vertices.size() / 3; i++) {
        v_rational.push_back(Vector3r());
#ifdef USE_GNU_GMP_CLASSES
        v_rational.back()[0].init(embedded_vertices[3 * i + 0].get_mpq_t());
        v_rational.back()[1].init(embedded_vertices[3 * i + 1].get_mpq_t());
        v_rational.back()[2].init(embedded_vertices[3 * i + 2].get_mpq_t());
#else
        v_rational.back()[0].init_from_bin(embedded_vertices[3 * i + 0].get_str());
        v_rational.back()[1].init_from_bin(embedded_vertices[3 * i + 1].get_str());
        v_rational.back()[2].init_from_bin(embedded_vertices[3 * i + 2].get_str());
#endif
    }

    // Debug-only sanity check: the remesher now returns tets already in the WMTK
    // orientation ((v1-v0)x(v2-v0).(v3-v0) > 0), so out_tets is used directly (no
    // orientation fix-up when filling tets_after below). This verification is
    // exact-rational and O(#tets) -- prohibitively expensive on large meshes --
    // so it is compiled out of release builds.
    if (opts.check_orientation) {
        logger().info("Check tet orientation after embedding...");
        for (const auto& vids : out_tets) {
            Vector3r n = (v_rational[vids[1]] - v_rational[vids[0]])
                             .cross(v_rational[vids[2]] - v_rational[vids[0]]);
            Vector3r d = v_rational[vids[3]] - v_rational[vids[0]];
            auto res = n.dot(d);
            if (res > 0) {
                continue;
            }
            logger().error(
                "After embed_tri_in_poly_mesh: Tet {} is inverted! res = {}",
                vids,
                res.to_double());
            for (size_t i = 0; i < vids.size(); ++i) {
                logger().error("v{} = {}", i, to_double(v_rational[vids[i]]).transpose());
            }
        }
        logger().info("done");
    }


    // Step 4b: decode embedded_facets into triangles.
    // here every facet must already be a triangle, so the array has a fixed
    // stride of 4 (1 size prefix + 3 vertex ids). If the remesher ever returns a
    // non-triangular facet this throws rather than triangulating it.
    logger().info("Facets loop...");
    polygon_faces.reserve(embedded_facets.size() / 4);
    for (size_t i = 0; i < embedded_facets.size(); i += 4) {
        const size_t polysize = embedded_facets[i];
        if (polysize != 3) {
            log_and_throw_error("Facets must be triangles!");
        }
        std::array<size_t, 3> polygon;
        for (size_t j = 0; j < 3; ++j) {
            polygon[j] = embedded_facets[j + i + 1];
        }
        polygon_faces.push_back(polygon);
    }
    logger().info("done");

    // Per-face on-input-surface flags.
    logger().info("Tags loop...");
    polygon_faces_on_input.assign(polygon_faces.size(), false);
    for (size_t i = 0; i < embedded_facets_on_input.size(); ++i) {
        polygon_faces_on_input[embedded_facets_on_input[i]] = true;
    }
    logger().info("done");

    // Step 4c: surface tracking, from the remesher-provided metadata. For each
    // output tet it looks at its parent cell (final_tets_parent) and the parent
    // polygon faces bounding it (final_tets_parent_faces), and marks the
    // corresponding local tet face.
    logger().info("tracking surface...");
    assert(final_tets_parent_faces.size() == out_tets.size());
    for (size_t i = 0; i < out_tets.size(); ++i) {
        const auto& tetra = out_tets[i];
        const uint32_t tetra_parent = final_tets_parent[i];

        // Fast path: if the parent cell has no faces on the input surface, none
        // of this tet's faces can either -- push four false flags and move on.
        if (!cells_with_faces_on_input[tetra_parent]) {
            for (int i = 0; i < 4; ++i) {
                tet_face_on_input_surface.push_back(false);
            }
            continue;
        }

        // The tet's four faces as sorted vertex sets, opposite each local vertex:
        // f0 opposite v0, f1 opposite v1, f2 opposite v2, f3 opposite v3. Sorting
        // lets us compare against each (also sorted) parent face by equality.
        // vector of std array and sort
        std::array<size_t, 3> local_f0{{tetra[1], tetra[2], tetra[3]}};
        std::sort(local_f0.begin(), local_f0.end());
        std::array<size_t, 3> local_f1{{tetra[0], tetra[2], tetra[3]}};
        std::sort(local_f1.begin(), local_f1.end());
        std::array<size_t, 3> local_f2{{tetra[0], tetra[1], tetra[3]}};
        std::sort(local_f2.begin(), local_f2.end());
        std::array<size_t, 3> local_f3{{tetra[0], tetra[1], tetra[2]}};
        std::sort(local_f3.begin(), local_f3.end());

        // For each parent face bounding this tet, find which local face it is and
        // copy that face's on-input flag into the matching slot.
        // track surface
        std::array<bool, 4> tet_face_on_input{{false, false, false, false}};
        for (const auto& f : final_tets_parent_faces[i]) {
            assert(polygon_faces[f].size() == 3);

            std::array<size_t, 3> f_vs = polygon_faces[f];
            std::sort(f_vs.begin(), f_vs.end());

            int64_t local_f_idx = -1;

            // decide which face it is

            if (f_vs == local_f0) {
                local_f_idx = 0;
            } else if (f_vs == local_f1) {
                local_f_idx = 1;
            } else if (f_vs == local_f2) {
                local_f_idx = 2;
            } else if (f_vs == local_f3) {
                local_f_idx = 3;
            }
            if (local_f_idx == -1) {
                log_and_throw_error("Could not find local index for tracked surface.");
            }

            tet_face_on_input[local_f_idx] = polygon_faces_on_input[f];
        }

        for (int k = 0; k < 4; k++) {
            tet_face_on_input_surface.push_back(tet_face_on_input[k]);
        }
    }

    // A vertex is on the input surface iff it is a corner of an on-input facet

    // track vertices on input
    is_v_on_input.resize(v_rational.size(), false);
    for (int i = 0; i < polygon_faces.size(); i++) {
        if (polygon_faces_on_input[i]) {
            is_v_on_input[polygon_faces[i][0]] = true;
            is_v_on_input[polygon_faces[i][1]] = true;
            is_v_on_input[polygon_faces[i][2]] = true;
        }
    }
    logger().info("done");

    // Step 4d: compact the vertex set. The arrangement may contain vertices not
    // referenced by any output tet, so drop the unused ones: build v_map (old id
    // -> new id) over the used vertices, rebuild the coord and on-input arrays in
    // the new numbering, then remap tets and facets.
    logger().info("removing unreferenced vertices...");
    std::vector<bool> v_is_used_in_tet(v_rational.size(), false);
    for (const auto& t : out_tets) {
        for (const auto& v : t) {
            v_is_used_in_tet[v] = true;
        }
    }
    std::vector<int64_t> v_map(v_rational.size(), -1);
    std::vector<Vector3r> v_coords_final;
    std::vector<bool> is_v_on_input_buffer;

    for (size_t i = 0; i < v_rational.size(); ++i) {
        if (v_is_used_in_tet[i]) {
            v_map[i] = v_coords_final.size();
            v_coords_final.emplace_back(v_rational[i]);
            is_v_on_input_buffer.emplace_back(is_v_on_input[i]);
        }
    }
    // update vertices
    v_rational = v_coords_final;
    is_v_on_input = is_v_on_input_buffer;
    // update tets (in place, into the compacted numbering)
    for (auto& t : out_tets) {
        for (int i = 0; i < 4; ++i) {
            assert(v_map[t[i]] >= 0);
            t[i] = v_map[t[i]];
        }
    }
    // update polygon_faces (in place, into the compacted numbering)
    for (auto& t : polygon_faces) {
        for (int i = 0; i < 3; ++i) {
            assert(v_map[t[i]] >= 0);
            t[i] = v_map[t[i]];
        }
    }
    logger().info("done");

    // Step 5: publish the tets. makeTetrahedra already emits WMTK-positively
    // oriented tets, so the vertices are copied straight through -- no swap. Only
    // the per-tet face flags need reordering: the tracking loop stored them in
    // "opposite-vertex" order (fl[k] = flag of the face opposite out_tets[i][k]),
    // which we map to WMTK's local face order.
    tets_after.resize(out_tets.size());
    for (size_t i = 0; i < out_tets.size(); ++i) {
        tets_after[i][0] = out_tets[i][0];
        tets_after[i][1] = out_tets[i][1];
        tets_after[i][2] = out_tets[i][2];
        tets_after[i][3] = out_tets[i][3];

        const bool fl0 = tet_face_on_input_surface[4 * i + 0]; // opp v0
        const bool fl1 = tet_face_on_input_surface[4 * i + 1]; // opp v1
        const bool fl2 = tet_face_on_input_surface[4 * i + 2]; // opp v2
        const bool fl3 = tet_face_on_input_surface[4 * i + 3]; // opp v3

        // WMTK local face order:
        //   local_f0: (v0, v1, v2) = opposite v3
        //   local_f1: (v0, v2, v3) = opposite v1
        //   local_f2: (v0, v1, v3) = opposite v2
        //   local_f3: (v1, v2, v3) = opposite v0
        tet_face_on_input_surface[4 * i + 0] = fl3;
        tet_face_on_input_surface[4 * i + 1] = fl1;
        tet_face_on_input_surface[4 * i + 2] = fl2;
        tet_face_on_input_surface[4 * i + 3] = fl0;
    }

    // final sanity check: every published tet must be positively
    // oriented under the WMTK convention ((v1-v0)x(v2-v0)).(v3-v0) > 0. Exact-
    // rational and O(#tets), so it is compiled out of release builds.
    if (opts.check_orientation) {
        logger().info("Check tet orientation after insertion...");
        for (const auto& vids : tets_after) {
            Vector3r n = (v_rational[vids[1]] - v_rational[vids[0]])
                             .cross(v_rational[vids[2]] - v_rational[vids[0]]);
            Vector3r d = v_rational[vids[3]] - v_rational[vids[0]];
            auto res = n.dot(d);
            if (res > 0) {
                continue;
            }
            logger().error("After insertion: Tet {} is inverted! res = {}", vids, res.to_double());
            for (size_t i = 0; i < vids.size(); ++i) {
                logger().error("v{} = {}", i, to_double(v_rational[vids[i]]).transpose());
            }
        }
        logger().info("done");
    }
}

} // namespace wmtk::utils
