#include "EmbedSurface.hpp"

// clang-format off
#include <igl/remove_unreferenced.h>
#include <igl/tet_tet_adjacency.h>
#include <igl/read_triangle_mesh.h>
#include <igl/winding_number.h>
// igl must be included BEFORE VolumeRemesher
#include <VolumeRemesher/embed.h>
#include <VolumeRemesher/numerics.h>
// clang-format on

#include <wmtk/utils/VectorUtils.h>
#include <bitset>
#include <filesystem>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/InsertTriangleUtils.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>

namespace {
std::vector<std::array<size_t, 3>> triangulate_polygon_face(std::vector<wmtk::Vector3r> points)
{
    using namespace wmtk;

    // triangulate weak convex polygons
    std::vector<std::array<size_t, 3>> triangulated_faces;

    std::vector<std::pair<Vector3r, int>> points_vector;
    for (int i = 0; i < points.size(); i++) {
        points_vector.push_back(std::pair<Vector3r, int>(points[i], i));
    }

    // find the first colinear ABC with nonlinear BCD and delete C from vector
    while (points_vector.size() > 3) {
        bool no_colinear = true;
        for (int i = 0; i < points_vector.size(); i++) {
            auto cur = points_vector[i];
            auto next = points_vector[(i + 1) % points_vector.size()];
            auto prev = points_vector[(i + points_vector.size() - 1) % points_vector.size()];
            auto nextnext = points_vector[(i + 2) % points_vector.size()];

            Vector3r a = cur.first - prev.first;
            Vector3r b = next.first - cur.first;
            Vector3r c = nextnext.first - next.first;

            if (((a[0] * b[1] - a[1] * b[0]) == 0 && (a[1] * b[2] - a[2] * b[1]) == 0 &&
                 (a[0] * b[2] - a[2] * b[0]) == 0) &&
                (((b[0] * c[1] - b[1] * c[0]) != 0 || (b[1] * c[2] - b[2] * c[1]) != 0 ||
                  (b[0] * c[2] - b[2] * c[0]) != 0))) {
                no_colinear = false;
                std::array<size_t, 3> t = {
                    {size_t(cur.second), size_t(next.second), size_t(nextnext.second)}};
                triangulated_faces.push_back(t);
                points_vector.erase(points_vector.begin() + ((i + 1) % points_vector.size()));
                break;
            } else {
                continue;
            }
        }

        if (no_colinear) {
            break;
        }
    }

    // cleanup convex polygon
    while (points_vector.size() >= 3) {
        std::array<size_t, 3> t = {
            {size_t(points_vector[0].second),
             size_t(points_vector[1].second),
             size_t(points_vector[points_vector.size() - 1].second)}};
        triangulated_faces.push_back(t);
        points_vector.erase(points_vector.begin());
    }

    return triangulated_faces;
}
} // namespace

namespace wmtk::components::simwild {

void delaunay_box_mesh(
    const wmtk::SampleEnvelope& envelope,
    const MatrixXd& vertices,
    std::vector<wmtk::delaunay::Point3D>& points,
    std::vector<wmtk::delaunay::Tetrahedron>& tets,
    Vector3d& box_min,
    Vector3d& box_max)
{
    assert(vertices.cols() == 3);

    const Vector3d vertices_max = vertices.colwise().maxCoeff();
    const Vector3d vertices_min = vertices.colwise().minCoeff();
    const double diag = (vertices_max - vertices_min).norm();

    ///points for delaunay
    points.resize(vertices.rows());
    for (int i = 0; i < vertices.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            points[i][j] = vertices(i, j);
        }
    }
    // bbox
    double delta = diag / 15.0;
    box_min = Vector3d(vertices_min[0] - delta, vertices_min[1] - delta, vertices_min[2] - delta);
    box_max = Vector3d(vertices_max[0] + delta, vertices_max[1] + delta, vertices_max[2] + delta);

    // add corners of domain
    for (int i = 0; i < 8; i++) {
        Vector3d p;
        std::bitset<sizeof(int) * 8> a(i);
        for (int j = 0; j < 3; j++) {
            if (a.test(j)) {
                p[j] = box_max[j];
            } else {
                p[j] = box_min[j];
            }
        }
        points.push_back({{p[0], p[1], p[2]}});
    }

    const double voxel_resolution = diag / 20.0;
    std::array<int, 3> N; // number of grid points per dimension
    std::array<double, 3> h; // distance between grid points per dimension
    for (int i = 0; i < 3; i++) {
        const double D = box_max[i] - box_min[i];
        N[i] = (D / voxel_resolution) + 1;
        h[i] = D / N[i];
    }

    std::array<std::vector<double>, 3> ds;
    for (int i = 0; i < 3; i++) {
        ds[i].push_back(box_min[i]);
        for (int j = 0; j < N[i] - 1; j++) {
            ds[i].push_back(box_min[i] + h[i] * (j + 1));
        }
        ds[i].push_back(box_max[i]);
    }

    const double min_dis = voxel_resolution * voxel_resolution / 4;
    //    double min_dis = state.target_edge_len * state.target_edge_len;//epsilon*2
    for (int i = 0; i < ds[0].size(); i++) {
        for (int j = 0; j < ds[1].size(); j++) {
            for (int k = 0; k < ds[2].size(); k++) {
                if ((i == 0 || i == ds[0].size() - 1) && (j == 0 || j == ds[1].size() - 1) &&
                    (k == 0 || k == ds[2].size() - 1)) {
                    continue;
                }
                const Vector3d p(ds[0][i], ds[1][j], ds[2][k]);

                Eigen::Vector3d n;
                const double sqd = envelope.nearest_point(p, n);

                if (sqd < min_dis) {
                    continue;
                }
                points.push_back({{ds[0][i], ds[1][j], ds[2][k]}});
            }
        }
    }

    ///delaunay
    std::vector<wmtk::delaunay::Point3D> unused_points;
    std::tie(unused_points, tets) = wmtk::delaunay::delaunay3D(points);
    wmtk::logger().info(
        "after delauney tets.size() = {} | points.size() = {}",
        tets.size(),
        points.size());
}

void embed_surface(
    const MatrixXd& V_surface,
    const MatrixXi& F_surface,
    const MatrixXd& V_vol,
    const MatrixXi& T_vol,
    MatrixXr& V_emb,
    MatrixXi& T_emb,
    MatrixXi& F_on_surface,
    const bool perform_sanity_checks)
{
    // old output
    std::vector<std::array<size_t, 3>> facets_after;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets_after;
    std::vector<bool> tet_face_on_input_surface;
    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> polygon_faces;

    ////
    std::vector<double> tet_vrt_coord(V_vol.size());
    std::vector<uint32_t> tet_indices(T_vol.size());

    for (int i = 0; i < V_vol.rows(); ++i) {
        tet_vrt_coord[3 * i + 0] = V_vol(i, 0);
        tet_vrt_coord[3 * i + 1] = V_vol(i, 1);
        tet_vrt_coord[3 * i + 2] = V_vol(i, 2);
    }

    for (int i = 0; i < T_vol.rows(); ++i) {
        tet_indices[4 * i + 0] = T_vol(i, 0);
        tet_indices[4 * i + 1] = T_vol(i, 1);
        tet_indices[4 * i + 2] = T_vol(i, 2);
        tet_indices[4 * i + 3] = T_vol(i, 3);
    }

    // Step 2 (cont.): flatten the input surface the same way.
    // prepare input surfaces info
    std::vector<double> tri_vrt_coord(V_surface.size());
    std::vector<uint32_t> triangle_indices(F_surface.size());

    for (int i = 0; i < V_surface.rows(); ++i) {
        tri_vrt_coord[3 * i + 0] = V_surface(i, 0);
        tri_vrt_coord[3 * i + 1] = V_surface(i, 1);
        tri_vrt_coord[3 * i + 2] = V_surface(i, 2);
    }

    for (int i = 0; i < F_surface.rows(); ++i) {
        triangle_indices[3 * i + 0] = (uint32_t)F_surface(i, 0);
        triangle_indices[3 * i + 1] = (uint32_t)F_surface(i, 1);
        triangle_indices[3 * i + 2] = (uint32_t)F_surface(i, 2);
    }

    // Remesher outputs. Unlike "_old", this variant actually uses the tet-based
    // outputs: out_tets (the remesher's tetrahedra), final_tets_parent (parent
    // polyhedral cell of each tet), cells_with_faces_on_input (per-cell flag),
    // and final_tets_parent_faces (the parent faces bounding each tet). The
    // embedded_cells polygonal-cell decoding used by "_old" is not needed here.
    std::vector<vol_rem::bigrational> embedded_vertices;
    std::vector<uint32_t> embedded_facets;
    std::vector<uint32_t> embedded_cells;
    std::vector<uint32_t> embedded_facets_on_input;

    std::vector<std::array<uint32_t, 4>> out_tets;
    std::vector<uint32_t> final_tets_parent;
    std::vector<bool> cells_with_faces_on_input;
    std::vector<std::vector<uint32_t>> final_tets_parent_faces;

    // Step 3: run the exact arrangement (identical call to the "_old" variant).
    // volumeremesher embed
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
        true);

    // Step 4a: copy the arrangement vertices to exact rational Vector3r (same as
    // "_old"). No compaction yet -- unused vertices are pruned near the end.
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
    if (perform_sanity_checks) {
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

    // Step 4b: decode embedded_facets into triangles. KEY DIFFERENCE vs "_old":
    // here every facet must already be a triangle, so the array has a fixed
    // stride of 4 (1 size prefix + 3 vertex ids). If the remesher ever returns a
    // non-triangular facet this throws, rather than triangulating it like "_old".
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

    // Per-face on-input-surface flags, same as "_old".
    logger().info("Tags loop...");
    std::vector<bool> polygon_faces_on_input(polygon_faces.size(), false);
    for (size_t i = 0; i < embedded_facets_on_input.size(); ++i) {
        polygon_faces_on_input[embedded_facets_on_input[i]] = true;
    }
    logger().info("done");

    // Step 4c: surface tracking. KEY DIFFERENCE vs "_old": rather than matching
    // each cell's faces by unordered vertex sets after self-tetrahedralizing, it
    // uses the remesher-provided metadata. For each output tet it looks at its
    // parent cell (final_tets_parent) and the parent polygon faces bounding it
    // (final_tets_parent_faces), and marks the corresponding local tet face.
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
    // (same rule as "_old").
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

    // Step 4d: compact the vertex set. KEY DIFFERENCE vs "_old", which keeps all
    // vertices (and even adds centroids): here the remesher's arrangement may
    // contain vertices not referenced by any output tet, so drop the unused ones.
    // Build v_map (old id -> new id) over the used vertices, rebuild the coord
    // and on-input arrays in the new numbering, then remap tets and facets.
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
    if (perform_sanity_checks) {
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

    T_emb.resize(tets_after.size(), 4);
    for (int i = 0; i < tets_after.size(); ++i) {
        const auto& t = tets_after[i];
        T_emb.row(i) = Vector4i(t[0], t[1], t[2], t[3]);
    }

    V_emb.resize(v_rational.size(), 3);
    for (int i = 0; i < v_rational.size(); ++i) {
        V_emb.row(i) = v_rational[i];
    }

    // std::set<size_t> v_on_input;
    // for (int i = 0; i < polygon_faces.size(); ++i) {
    //     if (!polygon_faces_on_input[i]) {
    //         continue;
    //     }
    //     const auto& f = polygon_faces[i];
    //     v_on_input.insert(f[0]);
    //     v_on_input.insert(f[1]);
    //     v_on_input.insert(f[2]);
    // }

    // size_t n_face_on_input = 0;
    // for (const bool b : polygon_faces_on_input) {
    //     if (b) {
    //         ++n_face_on_input;
    //     }
    // }

    // F_on_surface.resize(n_face_on_input, 3);
    // n_face_on_input = 0;
    // for (int i = 0; i < polygon_faces.size(); ++i) {
    //     if (!polygon_faces_on_input[i]) {
    //         continue;
    //     }
    //     const auto& f = polygon_faces[i];
    //     F_on_surface.row(n_face_on_input++) = Vector3i(f[0], f[1], f[2]);
    // }

    F_on_surface.resize(polygon_faces.size(), 3);
    for (int i = 0; i < polygon_faces.size(); ++i) {
        const auto& f = polygon_faces[i];
        F_on_surface.row(i) = Vector3i(f[0], f[1], f[2]);
    }
}

void embed_surface_old(
    const MatrixXd& V_surface,
    const MatrixXi& F_surface,
    const MatrixXd& V_vol,
    const MatrixXi& T_vol,
    MatrixXr& V_emb,
    MatrixXi& T_emb,
    MatrixXi& F_on_surface,
    const bool perform_sanity_checks)
{
    // old output
    std::vector<std::array<size_t, 3>> facets_after;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets_after;
    std::vector<bool> tet_face_on_input_surface;
    std::vector<Vector3r> v_rational;

    ////
    std::vector<double> tet_vrt_coord(V_vol.size());
    std::vector<uint32_t> tet_index(T_vol.size());
    std::cout << "tetver size: " << V_vol.rows() << std::endl;
    std::cout << "tet size: " << T_vol.rows() << std::endl;

    for (int i = 0; i < V_vol.rows(); ++i) {
        tet_vrt_coord[3 * i + 0] = V_vol(i, 0);
        tet_vrt_coord[3 * i + 1] = V_vol(i, 1);
        tet_vrt_coord[3 * i + 2] = V_vol(i, 2);
    }

    for (int i = 0; i < T_vol.rows(); ++i) {
        tet_index[4 * i + 0] = T_vol(i, 0);
        tet_index[4 * i + 1] = T_vol(i, 1);
        tet_index[4 * i + 2] = T_vol(i, 2);
        tet_index[4 * i + 3] = T_vol(i, 3);
    }

    // prepare input surfaces info
    std::vector<double> tri_vrt_coord(V_surface.size());
    std::vector<uint32_t> triangle_indices(F_surface.size());

    for (int i = 0; i < V_surface.rows(); ++i) {
        tri_vrt_coord[3 * i + 0] = V_surface(i, 0);
        tri_vrt_coord[3 * i + 1] = V_surface(i, 1);
        tri_vrt_coord[3 * i + 2] = V_surface(i, 2);
    }

    for (int i = 0; i < F_surface.rows(); ++i) {
        triangle_indices[3 * i + 0] = F_surface(i, 0);
        triangle_indices[3 * i + 1] = F_surface(i, 1);
        triangle_indices[3 * i + 2] = F_surface(i, 2);
    }

    std::cout << tri_vrt_coord.size() << std::endl;
    std::cout << triangle_indices.size() << std::endl;
    std::cout << tet_vrt_coord.size() << std::endl;
    std::cout << tet_index.size() << std::endl;

    std::vector<vol_rem::bigrational> embedded_vertices;
    std::vector<uint32_t> embedded_facets;
    std::vector<uint32_t> embedded_cells;
    std::vector<uint32_t> embedded_facets_on_input;

    std::vector<std::array<uint32_t, 4>> out_tets;
    std::vector<uint32_t> final_tets_parent;
    std::vector<bool> cells_with_faces_on_input;
    std::vector<std::vector<uint32_t>> final_tets_parent_faces;

    // volumeremesher embed
    vol_rem::embed_tri_in_poly_mesh(
        tri_vrt_coord,
        triangle_indices,
        tet_vrt_coord,
        tet_index,
        embedded_vertices,
        embedded_facets,
        embedded_cells,
        out_tets,
        final_tets_parent,
        embedded_facets_on_input,
        cells_with_faces_on_input,
        final_tets_parent_faces,
        true);

    assert(embedded_vertices.size() % 3 == 0);
    v_rational.resize(embedded_vertices.size() / 3);
    for (int i = 0; i < embedded_vertices.size() / 3; i++) {
#ifdef USE_GNU_GMP_CLASSES
        v_rational[i][0].init(embedded_vertices[3 * i + 0].get_mpq_t());
        v_rational[i][1].init(embedded_vertices[3 * i + 1].get_mpq_t());
        v_rational[i][2].init(embedded_vertices[3 * i + 2].get_mpq_t());
#else
        v_rational[i][0].init_from_bin(embedded_vertices[3 * i + 0].get_str());
        v_rational[i][1].init_from_bin(embedded_vertices[3 * i + 1].get_str());
        v_rational[i][2].init_from_bin(embedded_vertices[3 * i + 2].get_str());
#endif
    }

    std::vector<std::vector<size_t>> polygon_faces;
    int polycnt = 0;
    for (int i = 0; i < embedded_facets.size(); i++) {
        int polysize = embedded_facets[i];
        std::vector<size_t> polygon;
        for (int j = i + 1; j <= i + polysize; j++) {
            polygon.push_back(embedded_facets[j]);
        }
        polycnt++;
        polygon_faces.push_back(polygon);
        i += polysize;
    }

    std::vector<std::vector<size_t>> polygon_cells;
    std::vector<std::array<size_t, 4>> tets_final;
    for (int i = 0; i < embedded_cells.size(); i++) {
        std::vector<size_t> polygon_cell;
        int cellsize = embedded_cells[i];
        for (int j = i + 1; j <= i + cellsize; j++) {
            polygon_cell.push_back(embedded_cells[j]);
        }
        polygon_cells.push_back(polygon_cell);
        i += cellsize;
    }

    std::cout << "polygon cells num: " << polygon_cells.size() << std::endl;

    std::vector<bool> polygon_faces_on_input_surface(polygon_faces.size(), false);

    for (int i = 0; i < embedded_facets_on_input.size(); i++) {
        polygon_faces_on_input_surface[embedded_facets_on_input[i]] = true;
    }

    std::vector<std::array<size_t, 3>> triangulated_faces;
    std::vector<bool> triangulated_faces_on_input;
    std::vector<std::vector<size_t>> map_poly_to_tri_face(polygon_faces.size());

    int poly_cnt = 0;

    // triangulate polygon faces
    for (int i = 0; i < polygon_faces.size(); i++) {
        // already clipped in other polygon
        if (map_poly_to_tri_face[i].size() != 0) {
            continue;
        }

        // new polygon face to clip
        std::vector<std::array<size_t, 3>> clipped_indices;
        std::vector<Vector3r> poly_coordinates;
        std::vector<size_t> polygon_face = polygon_faces[i];
        assert(polygon_face.size() >= 3);

        if (polygon_face.size() == 3) {
            // already a triangle
            std::array<size_t, 3> triangle_face = {
                {polygon_face[0], polygon_face[1], polygon_face[2]}};
            int idx = triangulated_faces.size();
            triangulated_faces.push_back(triangle_face);
            if (polygon_faces_on_input_surface[i]) {
                triangulated_faces_on_input.push_back(true);
            } else {
                triangulated_faces_on_input.push_back(false);
            }
            map_poly_to_tri_face[i].push_back(idx);
        } else {
            poly_cnt++;
            for (int j = 0; j < polygon_faces[i].size(); j++) {
                poly_coordinates.push_back(v_rational[polygon_face[j]]);
            }

            clipped_indices = triangulate_polygon_face(poly_coordinates);
            for (int j = 0; j < clipped_indices.size(); j++) {
                // need to map oldface index to new face indices
                std::array<size_t, 3> triangle_face = {
                    {polygon_face[clipped_indices[j][0]],
                     polygon_face[clipped_indices[j][1]],
                     polygon_face[clipped_indices[j][2]]}};
                int idx = triangulated_faces.size();
                triangulated_faces.push_back(triangle_face);

                // track input faces
                if (polygon_faces_on_input_surface[i]) {
                    triangulated_faces_on_input.push_back(true);
                } else {
                    triangulated_faces_on_input.push_back(false);
                }
                map_poly_to_tri_face[i].push_back(idx);
            }
        }
    }

    std::cout << "poly_cnt:" << poly_cnt << std::endl;
    std::cout << "finish triangulation" << std::endl;
    std::cout << "vertice before tetra num: " << v_rational.size() << std::endl;

    int was_tet_cnt = 0;
    for (int i = 0; i < polygon_cells.size(); i++) {
        auto polygon_cell = polygon_cells[i];

        // get polygon vertices
        std::vector<size_t> polygon_vertices;
        for (auto f : polygon_cell) {
            for (auto v : polygon_faces[f]) {
                polygon_vertices.push_back(v);
            }
        }
        wmtk::vector_unique(polygon_vertices);

        // compute number of triangle faces
        int num_faces = 0;
        for (auto f : polygon_cell) {
            num_faces += map_poly_to_tri_face[f].size();
        }

        // polygon already a tet
        if (num_faces == 4) {
            was_tet_cnt++;
            assert(polygon_vertices.size() == 4);
            // get the correct orientation here
            size_t v0 = polygon_faces[polygon_cell[0]][0];
            size_t v1 = polygon_faces[polygon_cell[0]][1];
            size_t v2 = polygon_faces[polygon_cell[0]][2];
            size_t v3;
            for (const size_t v : polygon_faces[polygon_cell[1]]) {
                if (v != v0 && v != v1 && v != v2) {
                    v3 = v;
                    break;
                }
            }

            std::array<size_t, 4> tetra = {{v0, v1, v2, v3}};

            // if inverted then fix the orientation
            Vector3r v0v1 = v_rational[v1] - v_rational[v0];
            Vector3r v0v2 = v_rational[v2] - v_rational[v0];
            Vector3r v0v3 = v_rational[v3] - v_rational[v0];
            if ((v0v1.cross(v0v2)).dot(v0v3) < 0) {
                tetra = {{v1, v0, v2, v3}};
            }

            // push the tet to final queue;
            tets_final.push_back(tetra);

            std::set<size_t> local_f1 = {tetra[0], tetra[1], tetra[2]};
            std::set<size_t> local_f2 = {tetra[0], tetra[2], tetra[3]};
            std::set<size_t> local_f3 = {tetra[0], tetra[1], tetra[3]};
            std::set<size_t> local_f4 = {tetra[1], tetra[2], tetra[3]};

            // track surface     need to be fixed
            std::array<bool, 4> tet_face_on_input;
            for (const size_t f : polygon_cell) {
                std::set<size_t> f_vs = {
                    polygon_faces[f][0],
                    polygon_faces[f][1],
                    polygon_faces[f][2]};

                int local_f_idx;

                // decide which face it is

                if (f_vs == local_f1) {
                    local_f_idx = 0;
                } else if (f_vs == local_f2) {
                    local_f_idx = 1;
                } else if (f_vs == local_f3) {
                    local_f_idx = 2;
                } else {
                    local_f_idx = 3;
                }

                tet_face_on_input[local_f_idx] = polygon_faces_on_input_surface[f];
            }

            for (int k = 0; k < 4; k++) {
                tet_face_on_input_surface.push_back(tet_face_on_input[k]);
            }
            continue;
        }

        // compute centroid
        Vector3r centroid(0, 0, 0);
        for (auto v : polygon_vertices) {
            centroid = centroid + v_rational[v];
        }
        centroid = centroid / polygon_vertices.size();

        // trahedralize
        size_t centroid_idx = v_rational.size();
        v_rational.push_back(centroid);

        for (const size_t f : polygon_cell) {
            for (const size_t t : map_poly_to_tri_face[f]) {
                std::array<size_t, 4> tetra = {
                    {triangulated_faces[t][0],
                     triangulated_faces[t][1],
                     triangulated_faces[t][2],
                     centroid_idx}};
                // check inverted tet and fix
                Vector3r v0v1 = v_rational[tetra[1]] - v_rational[tetra[0]];
                Vector3r v0v2 = v_rational[tetra[2]] - v_rational[tetra[0]];
                Vector3r v0v3 = v_rational[tetra[3]] - v_rational[tetra[0]];
                if ((v0v1.cross(v0v2)).dot(v0v3) < 0) {
                    tetra = {
                        {triangulated_faces[t][1],
                         triangulated_faces[t][0],
                         triangulated_faces[t][2],
                         centroid_idx}};
                }

                tets_final.push_back(tetra);
                tet_face_on_input_surface.push_back(triangulated_faces_on_input[t]);
                tet_face_on_input_surface.push_back(false);
                tet_face_on_input_surface.push_back(false);
                tet_face_on_input_surface.push_back(false);
            }
        }
    }

    std::cout << "polygon was tet num: " << was_tet_cnt << std::endl;
    std::cout << "vertices final num: " << v_rational.size() << std::endl;
    std::cout << "tets final num: " << tets_final.size() << std::endl;

    std::cout << "track face size: " << tet_face_on_input_surface.size() << std::endl;

    facets_after = triangulated_faces;
    tets_after = tets_final;

    T_emb.resize(tets_final.size(), 4);
    for (int i = 0; i < tets_final.size(); ++i) {
        const auto& t = tets_final[i];
        T_emb.row(i) = Vector4i(t[0], t[1], t[2], t[3]);
    }

    V_emb.resize(v_rational.size(), 3);
    for (int i = 0; i < v_rational.size(); ++i) {
        V_emb.row(i) = v_rational[i];
    }

    std::set<size_t> v_on_input;
    for (int i = 0; i < triangulated_faces.size(); ++i) {
        if (!triangulated_faces_on_input[i]) {
            continue;
        }
        const auto& f = triangulated_faces[i];
        v_on_input.insert(f[0]);
        v_on_input.insert(f[1]);
        v_on_input.insert(f[2]);
    }

    size_t n_face_on_input = 0;
    for (const bool b : triangulated_faces_on_input) {
        if (b) {
            ++n_face_on_input;
        }
    }

    F_on_surface.resize(n_face_on_input, 3);
    n_face_on_input = 0;
    for (int i = 0; i < triangulated_faces.size(); ++i) {
        if (!triangulated_faces_on_input[i]) {
            continue;
        }
        const auto& f = triangulated_faces[i];
        F_on_surface.row(n_face_on_input++) = Vector3i(f[0], f[1], f[2]);
    }


    std::cout << "v on surface vector size: " << v_rational.size() << std::endl;
    std::cout << "v on surface: " << v_on_input.size() << std::endl;
}


EmbedSurface::EmbedSurface(
    const std::vector<std::string>& img_filenames,
    const std::vector<Matrix4d>& img_transform,
    const double tol_rel,
    const double tol_abs)
    : m_img_filenames(img_filenames)
{
    assert(img_filenames.size() == img_transform.size());

    Vs.resize(m_img_filenames.size());
    Fs.resize(m_img_filenames.size());

    for (size_t i = 0; i < m_img_filenames.size(); ++i) {
        if (!std::filesystem::exists(m_img_filenames[i])) {
            log_and_throw_error("Input file {} does not exist", m_img_filenames[i]);
        }
        MatrixXi F_single;
        io::read_triangle_mesh(m_img_filenames[i], Vs[i], F_single, tol_rel, tol_abs);

        assert(Vs[i].cols() == 3);
        assert(F_single.cols() == 3);

        // apply transform to Vs[i]
        for (size_t j = 0; j < Vs[i].rows(); ++j) {
            Vector3d v = Vs[i].row(j);
            Vector4d x = to_homogenuous(v);
            x = img_transform[i] * x;
            Vs[i].row(j) = from_homogenuous(x);
        }

        Fs[i] = F_single;

        const size_t nV_old = m_V_surface.rows();
        const size_t nF_old = m_F_surface.rows();

        m_V_surface.conservativeResize(m_V_surface.rows() + Vs[i].rows(), 3);
        m_V_surface.block(nV_old, 0, Vs[i].rows(), 3) = Vs[i];

        F_single.array() += nV_old;
        m_F_surface.conservativeResize(m_F_surface.rows() + F_single.rows(), 3);
        m_F_surface.block(nF_old, 0, Fs[i].rows(), 3) = F_single;
    }


    // process triangle soup
    MatrixXd V;
    MatrixXi F;
    VectorXi _I;

    // remove unreferenced vertices
    igl::remove_unreferenced(m_V_surface, m_F_surface, V, F, _I);

    if (V.rows() == 0 || F.rows() == 0) {
        log_and_throw_error("Empty Input");
    }

    wmtk::logger().info("All inputs #V = {}, #F = {}", V.rows(), F.rows());

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    VF_to_vectors(V, F, verts, tris);

    V_surf_from_vector(verts);
    F_surf_from_vector(tris);
}

void EmbedSurface::simplify_surface(const double eps, const int num_threads)
{
    // convert to STL vectors
    std::vector<Eigen::Vector3d> verts = V_surf_to_vector();
    std::vector<std::array<size_t, 3>> tris = F_surf_to_vector();

    shortest_edge_collapse::ShortestEdgeCollapse surf_mesh(verts, num_threads, false);

    // must be a small envelope to ensure correct tet tags later on
    surf_mesh.create_mesh(verts.size(), tris, modified_nonmanifold_v, eps);
    assert(surf_mesh.check_mesh_connectivity_validity());

    surf_mesh.collapse_shortest(0);
    surf_mesh.consolidate_mesh();
    // surf_mesh.write_triangle_mesh("triangle_soup_coarse.off");

    //// get the simplified input
    std::vector<Eigen::Vector3d> v_simplified;
    std::vector<std::array<size_t, 3>> f_simplified;
    v_simplified.resize(surf_mesh.vert_capacity());
    f_simplified.resize(surf_mesh.tri_capacity());
    for (const auto& t : surf_mesh.get_vertices()) {
        const size_t i = t.vid(surf_mesh);
        v_simplified[i] = surf_mesh.vertex_attrs[i].pos;
    }

    for (const auto& t : surf_mesh.get_faces()) {
        const auto i = t.fid(surf_mesh);
        const auto vs = surf_mesh.oriented_tri_vids(t);
        for (int j = 0; j < 3; j++) {
            f_simplified[i][j] = vs[j];
        }
    }

    V_surf_from_vector(v_simplified);
    F_surf_from_vector(f_simplified);
}

void EmbedSurface::remove_duplicates(const double eps)
{
    auto v = V_surf_to_vector();
    auto f = F_surf_to_vector();

    wmtk::remove_duplicates(v, f, eps);

    V_surf_from_vector(v);
    F_surf_from_vector(f);
}

bool EmbedSurface::embed_surface(const bool flood_fill)
{
    logger().info("Embed with VolumeInsertion");

    double eps = 0.5;
    if (!Fs.empty()) {
        const std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax =
            std::pair(m_V_surface.colwise().minCoeff(), m_V_surface.colwise().maxCoeff());
        double diag = (box_minmax.first - box_minmax.second).norm();
        eps = 1e-2 * diag;
    }

    std::shared_ptr<SampleEnvelope> ptr_env;
    {
        const auto v_simplified = V_surf_to_vector();

        std::vector<Eigen::Vector3i> tempF(m_F_surface.rows());
        for (size_t i = 0; i < tempF.size(); ++i) {
            tempF[i] = m_F_surface.row(i);
        }
        ptr_env = std::make_shared<SampleEnvelope>();
        ptr_env->use_exact = true;
        ptr_env->init(v_simplified, tempF, eps);
    }

    std::vector<wmtk::delaunay::Point3D> points;
    std::vector<wmtk::delaunay::Tetrahedron> tets;
    Vector3d box_min;
    Vector3d box_max;
    delaunay_box_mesh(*ptr_env, m_V_surface, points, tets, box_min, box_max);

    MatrixXd V_vol;
    V_vol.resize(points.size(), 3);
    for (int i = 0; i < points.size(); ++i) {
        const auto& v = points[i];
        V_vol.row(i) = Vector3d(v[0], v[1], v[2]);
    }

    MatrixXi T_vol;
    T_vol.resize(tets.size(), 4);
    for (int i = 0; i < tets.size(); ++i) {
        const auto& t = tets[i];
        T_vol.row(i) = Vector4i(t[0], t[1], t[2], t[3]);
    }

    simwild::embed_surface(
        m_V_surface,
        m_F_surface,
        V_vol,
        T_vol,
        m_V_emb_r,
        m_T_emb,
        m_F_on_surface,
        m_perform_sanity_checks);

    if (m_perform_sanity_checks) {
        // check that all tets contain valid vertex indices and no duplicates
        logger().info("Check for duplicate vertices...");
        for (int i = 0; i < m_T_emb.rows(); ++i) {
            const Vector4i& t = m_T_emb.row(i);
            std::set<int> s{t[0], t[1], t[2], t[3]};
            if (s.size() != 4) {
                log_and_throw_error("Tet {} has duplicate vertices: {}", i, t);
            }
            for (int j = 0; j < 4; ++j) {
                if (t[j] < 0 || t[j] >= m_V_emb_r.rows()) {
                    log_and_throw_error(
                        "Tet {} has invalid vertex index {} (out of range [0, {})",
                        i,
                        t[j],
                        m_V_emb_r.rows());
                }
            }
        }

        // check that all surface faces contain valid vertex indices and no duplicates
        for (int i = 0; i < m_F_on_surface.rows(); ++i) {
            const Vector3i& f = m_F_on_surface.row(i);
            std::set<int> s{f[0], f[1], f[2]};
            if (s.size() != 3) {
                log_and_throw_error("Surface face {} has duplicate vertices: {}", i, f);
            }
            for (int j = 0; j < 3; ++j) {
                if (f[j] < 0 || f[j] >= m_V_emb_r.rows()) {
                    log_and_throw_error(
                        "Surface face {} has invalid vertex index {} (out of range [0, {})",
                        i,
                        f[j],
                        m_V_emb_r.rows());
                }
            }
        }
        logger().info("done");

        // // check that all vertices are used in at least one tet
        // logger().info("Check that all vertices are used in at least one tet...");
        // std::vector<bool> vertex_used(m_V_emb_r.rows(), false);
        // for (int i = 0; i < m_T_emb.rows(); ++i) {
        //     const Vector4i& t = m_T_emb.row(i);
        //     for (int j = 0; j < 4; ++j) {
        //         vertex_used[t[j]] = true;
        //     }
        // }

        // for (int i = 0; i < vertex_used.size(); ++i) {
        //     if (!vertex_used[i]) {
        //         log_and_throw_error("Vertex {} is not used in any tet", i);
        //     }
        // }
        // logger().info("done");

        // check for degenerate tets (zero or negative volume)
        logger().info("Check for degenerate tets...");
        for (int i = 0; i < m_T_emb.rows(); ++i) {
            const Vector4i& t = m_T_emb.row(i);
            const Vector3r& v0 = m_V_emb_r.row(t[0]);
            const Vector3r& v1 = m_V_emb_r.row(t[1]);
            const Vector3r& v2 = m_V_emb_r.row(t[2]);
            const Vector3r& v3 = m_V_emb_r.row(t[3]);
            Vector3r v0v1 = v1 - v0;
            Vector3r v0v2 = v2 - v0;
            Vector3r v0v3 = v3 - v0;
            if ((v0v1.cross(v0v2)).dot(v0v3) <= 0) {
                log_and_throw_error(
                    "Tet {} is degenerate (zero or negative volume): vids = {}",
                    i,
                    t);
            }
        }
        logger().info("done");

        // check that each tet appears only once in the tet list
        logger().info("Check for duplicate tets...");
        std::set<std::set<int>> tet_set;
        for (int i = 0; i < m_T_emb.rows(); ++i) {
            const Vector4i& t = m_T_emb.row(i);
            std::set<int> s{t[0], t[1], t[2], t[3]};
            if (tet_set.count(s) > 0) {
                log_and_throw_error("Tet {} is a duplicate: {}", i, t);
            }
            tet_set.insert(s);
        }
        logger().info("done");

        // check that faces appear at most twice in the tet list (once for each adjacent tet)
        logger().info("Check for duplicate faces...");
        std::map<std::set<int>, int> face_count;
        for (int i = 0; i < m_T_emb.rows(); ++i) {
            const Vector4i& t = m_T_emb.row(i);
            std::array<std::set<int>, 4> faces = {{
                std::set<int>{t[0], t[1], t[2]},
                std::set<int>{t[0], t[1], t[3]},
                std::set<int>{t[1], t[2], t[3]},
                std::set<int>{t[2], t[0], t[3]},
            }};
            for (const auto& f : faces) {
                if (face_count.count(f) == 0) {
                    face_count[f] = 0;
                }
                face_count[f]++;
                if (face_count[f] > 2) {
                    log_and_throw_error("Face {} appears more than twice in the tet list", f);
                }
            }
        }
        logger().info("done");
    }

    // check for consistent orientation of tets
    logger().info("Check for consistent orientation between tets...");
    std::set<std::array<int, 3>> face_set; // each face must be unique
    for (int i = 0; i < m_T_emb.rows(); ++i) {
        const Vector4i& t = m_T_emb.row(i);
        std::array<std::array<int, 3>, 4> faces = {{
            {{t[1], t[3], t[2]}}, // opposite t[0]
            {{t[0], t[2], t[3]}}, // opposite t[1]
            {{t[0], t[3], t[1]}}, // opposite t[2]
            {{t[0], t[1], t[2]}}, // opposite t[3]
        }};
        // Rotate vertex IDs in faces to start with the lowest vertex ID. This ensures that the
        // same face is represented by the same set of vertex IDs, regardless of the order in
        // which they appear in the tet.
        for (auto& f : faces) {
            std::rotate(f.begin(), std::min_element(f.begin(), f.end()), f.end());
        }

        for (const auto& f : faces) {
            if (face_set.count(f) > 0) {
                log_and_throw_error("Face {} appears more than once in the tet list", f);
            }
            face_set.insert(f);
        }
    }
    logger().info("done");

    // const bool all_rounded = VF_rational_to_double(m_V_emb_r, m_T_emb, m_V_emb);
    // if (!all_rounded) {
    //     // log_and_throw_error("Tets are inverted after converting to double precision.");
    //     logger().info("Not all vertices can be rounded to double precision.");
    // }

    m_V_emb.resizeLike(m_V_emb_r);
    for (int i = 0; i < m_V_emb_r.size(); ++i) {
        m_V_emb(i) = m_V_emb_r(i).to_double();
    }
    /**
     * Only do a trivial rounding here. The more complex rounding is performed later on.
     */
    bool all_rounded = true;
    for (int i = 0; i < m_V_emb.rows(); ++i) {
        const Vector3r& r = m_V_emb_r.row(i);
        const Vector3r r_from_d = to_rational(Vector3d(m_V_emb.row(i)));
        if (r != r_from_d) {
            all_rounded = false;
            break;
        }
    }

    // add tags
    if (Fs.empty()) {
        log_and_throw_error("No input surface to embed");
    }
    tag_from_winding_number();

    /**
     * Cluster tags by flood-filling regions that are bounded by the surface. All tags within one
     * region are unified by taking the tag with most occurances.
     *
     * This is for several reasons not the best way to do this.
     * 1. The tags should be volume-weighted so that the tag that represents the most volume should
     * be picked.
     * 2. We should not rely on geometric look-up at all, but use the information stored in
     * m_F_tags_surface. This is more work so I took a shortcut here.
     */
    if (flood_fill) {
        logger().info("Use flood fill to unify tags.");
        std::set<simplex::Face> surface;
        for (size_t i = 0; i < m_F_on_surface.rows(); ++i) {
            const simplex::Face f(m_F_on_surface(i, 0), m_F_on_surface(i, 1), m_F_on_surface(i, 2));
            surface.insert(f);
        }

        // flood fill regions
        std::vector<bool> visited(m_T_emb.rows(), false);
        MatrixXi TT;
        igl::tet_tet_adjacency(m_T_emb, TT);

        for (size_t i = 0; i < m_T_emb.rows(); ++i) {
            if (visited[i]) {
                continue;
            }

            std::queue<size_t> q;
            q.push(i);
            visited[i] = true;

            std::vector<size_t> region; // all tets in that region

            while (!q.empty()) {
                size_t tid = q.front();
                q.pop();

                region.push_back(tid);
                const auto& tet = m_T_emb.row(tid);

                // face-vertex IDs according to igl::tet_tet_adjacency
                std::array<simplex::Face, 4> f{{
                    simplex::Face(tet[0], tet[1], tet[2]),
                    simplex::Face(tet[0], tet[1], tet[3]),
                    simplex::Face(tet[1], tet[2], tet[3]),
                    simplex::Face(tet[2], tet[0], tet[3]),
                }};
                for (size_t j = 0; j < 4; ++j) {
                    if (TT(tid, j) < 0) {
                        continue; // boundary
                    }
                    if (surface.count(f[j]) == 0 && !visited[TT(tid, j)]) {
                        visited[TT(tid, j)] = true;
                        q.push(TT(tid, j));
                    }
                }
            }

            // get all tags for one image
            for (size_t img_id = 0; img_id < Fs.size(); ++img_id) {
                std::map<int, int> m;
                for (size_t j = 0; j < region.size(); ++j) {
                    int tag = m_T_tags.coeff(region[j], img_id);
                    if (m.count(tag) == 0) {
                        m[tag] = 1;
                    } else {
                        m[tag] += 1;
                    }
                }
                int max_count = -1;
                int max_tag = -1;
                for (const auto& [tag, count] : m) {
                    if (count > max_count) {
                        max_count = count;
                        max_tag = tag;
                    }
                }
                assert(max_count > 0);
                for (size_t j = 0; j < region.size(); ++j) {
                    if (m_T_tags.coeff(region[j], img_id) != max_tag) {
                        m_T_tags.coeffRef(region[j], img_id) = max_tag;
                    }
                }
            }
        }
    }

    return all_rounded;
}

void EmbedSurface::consolidate()
{
    std::map<size_t, size_t> old2new;
    std::map<size_t, size_t> new2old;
    size_t new_vid_counter = 0;
    for (size_t i = 0; i < m_T_emb.rows(); ++i) {
        for (size_t j = 0; j < 4; ++j) {
            const auto vid = m_T_emb(i, j);
            if (old2new.count(vid) == 0) {
                old2new[vid] = new_vid_counter;
                new2old[new_vid_counter] = vid;
                ++new_vid_counter;
            }
        }
    }

    MatrixXd V;
    MatrixXr Vr;
    V.resize(new_vid_counter, 3);
    Vr.resize(new_vid_counter, 3);
    for (size_t i = 0; i < new_vid_counter; ++i) {
        V.row(i) = m_V_emb.row(new2old[i]);
        Vr.row(i) = m_V_emb_r.row(new2old[i]);
    }

    MatrixXi T;
    T.resizeLike(m_T_emb);
    for (size_t i = 0; i < m_T_emb.rows(); ++i) {
        for (size_t j = 0; j < 4; ++j) {
            T(i, j) = old2new[m_T_emb(i, j)];
        }
    }

    MatrixXi F_surf;
    F_surf.resizeLike(m_F_on_surface);
    for (size_t i = 0; i < m_F_on_surface.rows(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            F_surf(i, j) = old2new[m_F_on_surface(i, j)];
        }
    }

    m_V_emb = V;
    m_V_emb_r = Vr;
    m_T_emb = T;
    m_F_on_surface = F_surf;
}

void EmbedSurface::write_surf_off(const std::string& filename) const
{
    igl::writeOFF(filename, m_V_surface, m_F_surface);
}

void EmbedSurface::write_emb_surf_off(const std::string& filename) const
{
    igl::writeOFF(filename, m_V_emb, m_F_on_surface);
}

void EmbedSurface::write_emb_msh(const std::string& filename) const
{
    wmtk::MshData msh;
    msh.add_tet_vertices(m_V_emb.rows(), [this](size_t k) -> Vector3d { return m_V_emb.row(k); });

    const size_t n_tet_vertices = m_V_emb.rows();

    msh.add_tets(m_T_emb.rows(), [this](size_t k) { return m_T_emb.row(k); });

    for (size_t i = 0; i < m_T_tags.cols(); ++i) {
        msh.add_tet_attribute<1>(fmt::format("tag_{}", i), [this, i](size_t j) {
            return m_T_tags.coeff(j, i);
        });
    }

    msh.add_physical_group("ImageVolume");

    msh.add_face_vertices();
    msh.add_faces(m_F_on_surface.rows(), [this](size_t k) { return m_F_on_surface.row(k); });
    msh.add_physical_group("EmbeddedSurface");

    msh.add_face_vertices(m_V_surface.rows(), [this](size_t k) { return m_V_surface.row(k); });
    msh.add_faces(m_F_surface.rows(), [this](size_t k) { return m_F_surface.row(k); });
    msh.add_physical_group("EnvelopeSurface");

    msh.save(filename, true);
}

void EmbedSurface::write_emb_vtu(const std::string& filename) const
{
    paraviewo::VTUWriter writer;
    for (size_t i = 0; i < m_T_tags.cols(); ++i) {
        writer.add_cell_field(fmt::format("tag_{}", i), m_T_tags.col(i).cast<double>());
    }
    logger().info("Write {}.vtu and _surf.vtu", filename);
    writer.write_mesh(filename + ".vtu", m_V_emb, m_T_emb, paraviewo::CellType::Tetrahedron);
    writer
        .write_mesh(filename + "_surf.vtu", m_V_emb, m_F_on_surface, paraviewo::CellType::Triangle);
}

std::pair<Vector3d, Vector3d> EmbedSurface::bbox_minmax() const
{
    const Vector3d bbox_max = m_V_emb.colwise().maxCoeff();
    const Vector3d bbox_min = m_V_emb.colwise().minCoeff();

    return std::make_pair(bbox_min, bbox_max);
}

std::pair<Vector3d, Vector3d> EmbedSurface::bbox_surf_minmax() const
{
    const Vector3d bbox_max = m_V_surface.colwise().maxCoeff();
    const Vector3d bbox_min = m_V_surface.colwise().minCoeff();

    return std::make_pair(bbox_min, bbox_max);
}

std::vector<Eigen::Vector3d> EmbedSurface::V_surf_to_vector() const
{
    std::vector<Eigen::Vector3d> verts;

    verts.resize(m_V_surface.rows());
    for (size_t i = 0; i < m_V_surface.rows(); ++i) {
        verts[i] = m_V_surface.row(i);
    }

    return verts;
}

std::vector<std::array<size_t, 3>> EmbedSurface::F_surf_to_vector() const
{
    std::vector<std::array<size_t, 3>> tris;

    tris.resize(m_F_surface.rows());
    for (size_t i = 0; i < m_F_surface.rows(); ++i) {
        tris[i][0] = m_F_surface(i, 0);
        tris[i][1] = m_F_surface(i, 1);
        tris[i][2] = m_F_surface(i, 2);
    }

    return tris;
}

void EmbedSurface::V_surf_from_vector(const std::vector<Eigen::Vector3d>& verts)
{
    const V_MAP V_surface(verts[0].data(), verts.size(), 3);
    m_V_surface = V_surface;
    assert(m_V_surface.rows() == verts.size());
    assert(m_V_surface.cols() == 3);
}

void EmbedSurface::F_surf_from_vector(const std::vector<std::array<size_t, 3>>& tris)
{
    m_F_surface.resize(tris.size(), 3);
    for (size_t i = 0; i < tris.size(); ++i) {
        m_F_surface(i, 0) = tris[i][0];
        m_F_surface(i, 1) = tris[i][1];
        m_F_surface(i, 2) = tris[i][2];
    }
}

void EmbedSurface::tag_from_winding_number()
{
    m_T_tags.resize(m_T_emb.rows(), Fs.size());
    m_T_tags.setZero();
    m_tags.resize(m_T_emb.rows());

    MatrixXd P;
    P.resize(m_T_emb.rows(), 3);
    for (size_t i = 0; i < m_T_emb.rows(); ++i) {
        const Vector3d v0 = m_V_emb.row(m_T_emb(i, 0));
        const Vector3d v1 = m_V_emb.row(m_T_emb(i, 1));
        const Vector3d v2 = m_V_emb.row(m_T_emb(i, 2));
        const Vector3d v3 = m_V_emb.row(m_T_emb(i, 3));
        P.row(i) = (v0 + v1 + v2 + v3) * 0.25;
    }

    VectorXd W;
    W.resize(m_T_emb.rows());
    for (size_t i = 0; i < Fs.size(); ++i) {
        igl::winding_number(Vs[i], Fs[i], P, W);
        assert(W.size() == m_T_tags.rows());
        for (size_t j = 0; j < W.size(); ++j) {
            if (W[j] < 0.5) {
                continue;
            }
            m_T_tags.coeffRef(j, i) = 1;
            m_tags[j].insert((int64_t)i);
        }
    }
    m_T_tags.makeCompressed();
}

} // namespace wmtk::components::simwild