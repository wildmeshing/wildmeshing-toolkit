#include <wmtk/utils/EmbedTriangles.hpp>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/predicates.hpp>

// MatrixBase::cross is declared by Eigen/Core but only DEFINED in Eigen/Geometry, so without
// this the two normal computations below compile and then fail to link.
#include <Eigen/Geometry>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/VolumeRemesher.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <set>

namespace wmtk::utils {
namespace {

int sign_of(const Rational& r)
{
    return r.get_sign();
}

/**
 * The input's orientation on every on-input facet, as a signed count against the facet's
 * ascending vertex order -- see embed_triangles_in_tets' tet_face_orientation.
 *
 * `groups_of(facet)` yields the (facet, group) entries naming a facet. Within a group, the
 * input's signed coverage -- the sum over the group's triangles of +-1 where they cover -- is
 * constant over every facet tiling it: the arrangement cuts wherever it changes. It does NOT
 * necessarily cut along every input edge, though: the edge between two coplanar triangles of
 * a group need not be in the arrangement (on Thingi10K 104513 a facet straddles one, with its
 * centroid exactly on it), so a facet is not inside one triangle or another in general.
 *
 * Exact where it matters, cheap where it can be. Within a group every triangle's normal is
 * parallel to the group's reference normal, so a group whose triangles all face the same way
 * gives every facet tiling it the same answer, one sign test; that test is the sign of a dot
 * product of two parallel vectors, taken in double precision when the rounding cannot reach
 * it and exactly otherwise. (Such a group counts once even where its triangles overlap: a
 * region covered twice the same way is taken as one sheet.) A group whose triangles face both
 * ways -- a solid resting on another with a shared edge, a fold -- takes the exact path: the
 * coverage at a point of the facet that lies on no edge of the group's triangles, which, the
 * coverage being constant over the facet, is the facet's.
 */
template <typename GroupsOf>
std::vector<int> facet_orientations(
    const std::vector<double>& tri_vrt_coord,
    const std::vector<uint32_t>& triangle_indices,
    const std::vector<uint32_t>& tri_group,
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::array<size_t, 3>>& facets,
    const std::vector<bool>& facets_on_input,
    const GroupsOf& groups_of)
{
    const size_t n_tri = triangle_indices.size() / 3;
    const auto in_vertex = [&](uint32_t v) {
        Vector3r p;
        for (int k = 0; k < 3; ++k) p[k] = Rational(tri_vrt_coord[3 * v + k]);
        return p;
    };
    const auto tri_normal = [&](size_t t) {
        const Vector3r a = in_vertex(triangle_indices[3 * t + 0]);
        const Vector3r b = in_vertex(triangle_indices[3 * t + 1]);
        const Vector3r c = in_vertex(triangle_indices[3 * t + 2]);
        return Vector3r((b - a).cross(c - a));
    };

    // Per group: a reference normal, and whether all its triangles face along it.
    uint32_t n_groups = 0;
    for (const uint32_t g : tri_group) {
        if (g != UINT32_MAX) n_groups = std::max(n_groups, g + 1);
    }
    std::vector<std::vector<size_t>> group_tris(n_groups);
    for (size_t t = 0; t < n_tri; ++t) {
        if (tri_group[t] != UINT32_MAX) group_tris[tri_group[t]].push_back(t);
    }
    std::vector<Vector3r> ref(n_groups);
    std::vector<Vector3d> ref_unit(n_groups);
    std::vector<int> drop_axis(n_groups, 2);
    std::vector<bool> uniform(n_groups, true);
    std::vector<int> tri_sign(n_tri, 0); // against its group's reference normal
    for (uint32_t g = 0; g < n_groups; ++g) {
        if (group_tris[g].empty()) continue;
        ref[g] = tri_normal(group_tris[g][0]);
        const Vector3d r = to_double(ref[g]);
        ref_unit[g] = r.normalized();
        r.cwiseAbs().maxCoeff(&drop_axis[g]);
        for (const size_t t : group_tris[g]) {
            tri_sign[t] = sign_of(tri_normal(t).dot(ref[g]));
            if (tri_sign[t] != 1) uniform[g] = false;
        }
    }

    // sign(n_facet . ref_unit), where n_facet is the facet's normal in ascending vertex order.
    // n_facet is exactly parallel to the reference, so the dot is +-|n_facet| and its sign is
    // the answer; the double evaluation is trusted once it clears the rounding bound (input
    // coordinates rounded to double, then a cross product of differences of them).
    const auto facet_sign = [&](const std::array<size_t, 3>& f, uint32_t g) {
        const Vector3d a = to_double(v_rational[f[0]]);
        const Vector3d b = to_double(v_rational[f[1]]);
        const Vector3d c = to_double(v_rational[f[2]]);
        const double m =
            std::max({a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff(), c.cwiseAbs().maxCoeff()});
        const double err = 128 * DBL_EPSILON * m * m;
        const double dot = (b - a).cross(c - a).dot(ref_unit[g]);
        if (std::abs(dot) > 2 * err) return dot > 0 ? 1 : -1;
        const Vector3r n =
            (v_rational[f[1]] - v_rational[f[0]]).cross(v_rational[f[2]] - v_rational[f[0]]);
        return sign_of(n.dot(ref[g]));
    };

    // Where is the (rational) point q, on group g's plane, w.r.t. input triangle t? Exact:
    // 1 strictly inside, 0 outside, -1 on one of its edges (so q tells nothing about t).
    const auto classify = [&](const Vector3r& q, size_t t, uint32_t g) {
        const int ax0 = (drop_axis[g] + 1) % 3, ax1 = (drop_axis[g] + 2) % 3;
        std::array<Vector3r, 3> p;
        for (int j = 0; j < 3; ++j) p[j] = in_vertex(triangle_indices[3 * t + j]);
        const auto orient2 = [&](const Vector3r& u, const Vector3r& v, const Vector3r& w) {
            return sign_of(
                (v[ax0] - u[ax0]) * (w[ax1] - u[ax1]) - (v[ax1] - u[ax1]) * (w[ax0] - u[ax0]));
        };
        const auto between = [&](const Vector3r& u, const Vector3r& v) {
            for (const int ax : {ax0, ax1}) {
                const Rational& lo = u[ax] < v[ax] ? u[ax] : v[ax];
                const Rational& hi = u[ax] < v[ax] ? v[ax] : u[ax];
                if (q[ax] < lo || q[ax] > hi) return false;
            }
            return true;
        };
        const int s = orient2(p[0], p[1], p[2]);
        bool inside = true;
        for (int j = 0; j < 3; ++j) {
            const int o = orient2(p[j], p[(j + 1) % 3], q);
            if (o == 0 && between(p[j], p[(j + 1) % 3])) return -1;
            if (o != s) inside = false;
        }
        return inside ? 1 : 0;
    };

    // Interior points of a facet to try, as integer barycentric weights in general position:
    // the first one on no edge of the group's triangles is used.
    static constexpr int candidates[8][3] =
        {{1, 1, 1}, {5, 3, 2}, {2, 5, 3}, {3, 2, 5}, {7, 4, 2}, {2, 7, 4}, {4, 2, 7}, {11, 6, 5}};

    std::vector<int> res(facets.size(), 0);
    for (size_t i = 0; i < facets.size(); ++i) {
        if (!facets_on_input[i]) continue;
        std::array<size_t, 3> f = facets[i];
        std::sort(f.begin(), f.end());
        const auto [lo, hi] = groups_of(f);
        int o = 0;
        for (auto it = lo; it != hi; ++it) {
            const uint32_t g = it->second;
            const int fs = facet_sign(f, g);
            if (uniform[g]) {
                o += fs;
                continue;
            }
            bool found = false;
            for (const auto& w : candidates) {
                const Vector3r q =
                    (v_rational[f[0]] * Rational(w[0]) + v_rational[f[1]] * Rational(w[1]) +
                     v_rational[f[2]] * Rational(w[2])) /
                    Rational(w[0] + w[1] + w[2]);
                int cover = 0;
                bool on_edge = false;
                for (const size_t t : group_tris[g]) {
                    const int c = classify(q, t, g);
                    if (c < 0) {
                        on_edge = true;
                        break;
                    }
                    cover += c * tri_sign[t];
                }
                if (on_edge) continue;
                o += fs * cover;
                found = true;
                break;
            }
            if (!found) {
                logger().warn(
                    "orientation of facet {}: every test point lies on an input edge; left 0",
                    i);
            }
        }
        res[i] = o;
    }
    return res;
}

} // namespace
} // namespace wmtk::utils

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
    const EmbedTrianglesOptions& opts,
    EmbedTrianglesProvenance* provenance,
    std::vector<int>* tet_face_orientation)
{
    // Remesher outputs. The tet-based ones are what this consumes: out_tets (the
    // remesher's tetrahedra), final_tets_parent (parent polyhedral cell of each
    // tet), cells_with_faces_on_input (per-cell flag) and final_tets_parent_faces
    // (the parent faces bounding each tet). embedded_cells is not decoded.
    std::vector<NFG::bigrational> embedded_vertices;
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
    std::vector<uint32_t> vr_tri_group;
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
        vr_tri_group,
        vr_edge_provenance,
        vr_point_provenance,
        true);

    // Step 4a: copy the arrangement vertices to exact rational Vector3r. No
    // compaction yet -- unused vertices are pruned near the end.
    v_rational.reserve(v_rational.size() + embedded_vertices.size() / 3);
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
    // The remesher's numbers are all converted: free them, then return the memory NFG's
    // thread-local bignatural pool grew into while computing them. The pool never shrinks on
    // its own, so the arrangement's high-water mark would otherwise stay allocated for the rest
    // of the run (171 MB on Thingi10K 46024); later exact arithmetic regrows it only as far as
    // it needs. A no-op if any of this thread's bignaturals is still alive.
    std::vector<NFG::bigrational>().swap(embedded_vertices);
    NFG::bignatural::trimMemoryPool();

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

    // Per-face on-input-surface flags, from the remesher's triangle provenance.
    //
    // vr_tri_provenance[g] lists, for coplanar group g of the input, the output faces tiling
    // it as {tet, v0, v1, v2}; vr_tri_group maps each input triangle to its g. A face is on
    // the input surface iff some group names it. The faces are named by vertex triple rather
    // than by facet index, so index them the same way -- sorted triples in a sorted vector,
    // which is both cheaper to build than a hash set and deterministic.
    //
    // What this replaces is the remesher's `facets_on_input`, which is every face coloured
    // BLACK_A ("fully contained in one constraint"). The two agree on every model in the
    // suite (measured, see the commit message), but provenance is the stronger statement:
    // it is contained in a *genuine input* constraint, positive-area-overlapping it, whereas
    // the colour also fires on the virtual constraints the arrangement adds for itself.
    logger().info("Tags loop...");
    using FaceKey = std::pair<std::array<uint32_t, 3>, uint32_t>; // sorted triple -> group
    std::vector<FaceKey> on_input_faces;
    {
        size_t n = 0;
        for (const auto& group : vr_tri_provenance) {
            n += group.size();
        }
        on_input_faces.reserve(n);
        for (size_t g = 0; g < vr_tri_provenance.size(); ++g) {
            for (const auto& e : vr_tri_provenance[g]) {
                std::array<uint32_t, 3> key{{e[1], e[2], e[3]}};
                std::sort(key.begin(), key.end());
                on_input_faces.emplace_back(key, uint32_t(g));
            }
        }
        // A face where two exactly-coplanar groups meet is listed under both, so the keys
        // are unique as pairs but the triples are not.
        std::sort(on_input_faces.begin(), on_input_faces.end());
    }
    // All the entries naming one output face, as a range in the sorted vector above.
    const auto groups_of = [&on_input_faces](const std::array<size_t, 3>& f) {
        std::array<uint32_t, 3> key{{uint32_t(f[0]), uint32_t(f[1]), uint32_t(f[2])}};
        std::sort(key.begin(), key.end());
        return std::equal_range(
            on_input_faces.begin(),
            on_input_faces.end(),
            FaceKey{key, 0},
            [](const FaceKey& a, const FaceKey& b) { return a.first < b.first; });
    };

    polygon_faces_on_input.assign(polygon_faces.size(), false);
    if (provenance != nullptr) {
        provenance->triangle_group = vr_tri_group;
        provenance->face_groups.reserve(on_input_faces.size());
    }
    for (size_t i = 0; i < polygon_faces.size(); ++i) {
        const auto [lo, hi] = groups_of(polygon_faces[i]);
        polygon_faces_on_input[i] = lo != hi;
        if (provenance != nullptr) {
            for (auto it = lo; it != hi; ++it) {
                provenance->face_groups.push_back({uint32_t(i), it->second});
            }
        }
    }
    logger().info("done");

    // Per facet: the input's orientation on it, against the facet's ascending vertex order.
    // The remesher's facet vertex order says nothing about the input (it comes from whichever
    // tet's local face first named the facet), and its coplanar groups are unions over
    // undirected edges, so one group can hold triangles facing both ways. So: a facet tiling a
    // group counts +1 or -1 for every input triangle of that group it lies in, by whether its
    // normal agrees with that triangle's, and the counts of every group naming it add up.
    std::vector<int> facet_orientation;
    if (tet_face_orientation != nullptr) {
        logger().info("Orienting the tracked surface...");
        facet_orientation = facet_orientations(
            tri_vrt_coord,
            triangle_indices,
            vr_tri_group,
            v_rational,
            polygon_faces,
            polygon_faces_on_input,
            groups_of);
        logger().info("done");
    }

    // The old colour-based answer, kept only to be diffed against the one above. Off by
    // default: it is what the check exists to retire, and holding both costs a second pass.
    if (opts.check_surface_provenance) {
        std::vector<bool> by_colour(polygon_faces.size(), false);
        for (size_t i = 0; i < embedded_facets_on_input.size(); ++i) {
            by_colour[embedded_facets_on_input[i]] = true;
        }
        size_t only_colour = 0, only_provenance = 0;
        for (size_t i = 0; i < polygon_faces.size(); ++i) {
            if (by_colour[i] && !polygon_faces_on_input[i]) {
                ++only_colour;
            } else if (!by_colour[i] && polygon_faces_on_input[i]) {
                ++only_provenance;
            }
        }
        logger().info(
            "surface provenance check: {} faces on input by provenance, {} by colour, "
            "{} only-colour, {} only-provenance",
            std::count(polygon_faces_on_input.begin(), polygon_faces_on_input.end(), true),
            std::count(by_colour.begin(), by_colour.end(), true),
            only_colour,
            only_provenance);
    }

    // Step 4c: surface tracking, from the remesher-provided metadata. For each
    // output tet it looks at its parent cell (final_tets_parent) and the parent
    // polygon faces bounding it (final_tets_parent_faces), and marks the
    // corresponding local tet face.
    logger().info("tracking surface...");
    assert(final_tets_parent_faces.size() == out_tets.size());
    for (size_t i = 0; i < out_tets.size(); ++i) {
        const auto& tetra = out_tets[i];

        // Fast path: if none of the parent faces bounding this tet is on the input surface,
        // neither is any of its own faces -- push four false flags and skip the sorting
        // below. This is the exact test rather than the remesher's per-cell
        // cells_with_faces_on_input, which answers the same question one level coarser and
        // in terms of the face colour this no longer reads.
        bool any_on_input = false;
        for (const auto& f : final_tets_parent_faces[i]) {
            if (polygon_faces_on_input[f]) {
                any_on_input = true;
                break;
            }
        }
        if (!any_on_input) {
            for (int k = 0; k < 4; ++k) {
                tet_face_on_input_surface.push_back(false);
                if (tet_face_orientation != nullptr) tet_face_orientation->push_back(0);
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
        std::array<int, 4> tet_face_orient{{0, 0, 0, 0}};
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
            // Same vertex set as the facet, so the ascending-order value carries over as is.
            if (tet_face_orientation != nullptr)
                tet_face_orient[local_f_idx] = facet_orientation[f];
        }

        for (int k = 0; k < 4; k++) {
            tet_face_on_input_surface.push_back(tet_face_on_input[k]);
            if (tet_face_orientation != nullptr)
                tet_face_orientation->push_back(tet_face_orient[k]);
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
    // Sized up front and moved, not copied, into place: a Rational copy allocates fresh limbs,
    // so growing v_coords_final by doubling and then copy-assigning it back held up to three
    // copies of every exact coordinate at once.
    const size_t n_used = std::count(v_is_used_in_tet.begin(), v_is_used_in_tet.end(), true);
    v_coords_final.reserve(n_used);
    is_v_on_input_buffer.reserve(n_used);

    for (size_t i = 0; i < v_rational.size(); ++i) {
        if (v_is_used_in_tet[i]) {
            v_map[i] = v_coords_final.size();
            v_coords_final.emplace_back(v_rational[i]);
            is_v_on_input_buffer.emplace_back(is_v_on_input[i]);
        }
    }
    // update vertices
    v_rational = std::move(v_coords_final);
    is_v_on_input = std::move(is_v_on_input_buffer);
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

        // The orientations take the same reorder. They are measured against ascending vertex
        // ids, which the monotone compaction above (v_map is increasing) leaves in order.
        if (tet_face_orientation != nullptr) {
            auto& o = *tet_face_orientation;
            const std::array<int, 4> opp{{o[4 * i + 0], o[4 * i + 1], o[4 * i + 2], o[4 * i + 3]}};
            o[4 * i + 0] = opp[3];
            o[4 * i + 1] = opp[1];
            o[4 * i + 2] = opp[2];
            o[4 * i + 3] = opp[0];
        }
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
