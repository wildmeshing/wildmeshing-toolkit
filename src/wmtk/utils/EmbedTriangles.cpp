#include <wmtk/utils/EmbedTriangles.hpp>

#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/orient_by_majority.hpp>
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
#include <numeric>
#include <optional>
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
 * `groups_of(facet)` yields the (facet, group) entries naming a facet. Each coplanar group is
 * one SHEET: it contributes the sign of its coverage on the facet -- +1, -1 or 0, the coverage
 * being the sum over the group's triangles of +-1 where they cover -- and the contributions of
 * every group naming the facet add up. Taking the sign is what makes duplicated or overlapping
 * same-facing triangles of one group count once; distinct groups coinciding (two solids'
 * coplanar faces overlapping without sharing an edge) still add.
 *
 * Within a group the coverage is constant over every facet when the input is consistently
 * oriented: where it changes, some non-coplanar input triangle meets the plane, and the
 * arrangement cuts there. It does NOT cut along every input edge, though: the edge between two
 * coplanar triangles of a group lying on either side of it need not be in the arrangement (on
 * Thingi10K 104513 a facet straddles one, with its centroid exactly on it).
 *
 * Exact where it matters, cheap where it can be. Within a group every triangle's normal is
 * parallel to the group's reference normal, so a group whose triangles all face the same way
 * gives every facet tiling it the same answer, one sign test; that test is the sign of a dot
 * product of two parallel vectors, taken in double precision when the rounding cannot reach
 * it and exactly otherwise. A group whose triangles face both ways -- solids touching
 * face-to-face along a shared edge, ordinary in CAD assemblies, where the contact must cancel
 * to 0 -- takes the exact path: the coverage at a point of the facet that lies on no edge of
 * the group's triangles, with each triangle's exact vertices computed once and a bounding-box
 * test in front of every exact containment test.
 *
 * Where a flat region of the input is NOT consistently oriented -- a triangle flipped against
 * its coplanar neighbour -- the coverage can change across an edge the arrangement did not
 * cut, and a facet straddling it gets the coverage at whichever test point came first. Such
 * facets are noticed by a second test point disagreeing with the first, counted in
 * `n_straddling`, and left to orient_facet_patches to make consistent with their patch.
 */
template <typename GroupsOf>
std::vector<int> facet_orientations(
    const std::vector<double>& tri_vrt_coord,
    const std::vector<uint32_t>& triangle_indices,
    const std::vector<uint32_t>& tri_group,
    const size_t n_groups,
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::array<size_t, 3>>& facets,
    const std::vector<bool>& facets_on_input,
    const GroupsOf& groups_of,
    const int num_threads,
    size_t& n_straddling)
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

    // Per group: a reference normal, and whether all its triangles face along it. Sized by the
    // remesher's group count; a group no triangle names has no reference and contributes
    // nothing.
    std::vector<std::vector<size_t>> group_tris(n_groups);
    for (size_t t = 0; t < n_tri; ++t) {
        if (tri_group[t] < n_groups) group_tris[tri_group[t]].push_back(t);
    }
    std::vector<Vector3r> ref(n_groups);
    std::vector<Vector3d> ref_unit(n_groups, Vector3d::Zero());
    std::vector<int> drop_axis(n_groups, 2);
    std::vector<bool> uniform(n_groups, true);
    std::vector<int> tri_sign(n_tri, 0); // against its group's reference normal
    for (size_t g = 0; g < n_groups; ++g) {
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

    // Mixed groups only: each triangle projected along the group's dropped axis, exactly, with
    // its winding sign there and its bounding box in double. The input coordinates are doubles,
    // so the box is exact; the test against it widens by a relative margin far above the
    // rounding of the query point, so a point the box rejects is outside the triangle and off
    // its edges too.
    struct MixedTri
    {
        std::array<std::array<Rational, 2>, 3> p;
        int winding; // orientation of p0 p1 p2 in the projection plane
        int sign; // the triangle's normal against the group's reference
        std::array<double, 4> box; // lo0, hi0, lo1, hi1
    };
    std::vector<std::vector<MixedTri>> mixed(n_groups);
    const auto orient2 = [](const std::array<Rational, 2>& u,
                            const std::array<Rational, 2>& v,
                            const std::array<Rational, 2>& w) {
        return sign_of((v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0]));
    };
    for (size_t g = 0; g < n_groups; ++g) {
        if (uniform[g] || group_tris[g].empty()) continue;
        const int ax0 = (drop_axis[g] + 1) % 3, ax1 = (drop_axis[g] + 2) % 3;
        mixed[g].reserve(group_tris[g].size());
        for (const size_t t : group_tris[g]) {
            MixedTri mt;
            mt.box = {{DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX}};
            for (int j = 0; j < 3; ++j) {
                const uint32_t v = triangle_indices[3 * t + j];
                const double x0 = tri_vrt_coord[3 * v + ax0], x1 = tri_vrt_coord[3 * v + ax1];
                mt.p[j] = {{Rational(x0), Rational(x1)}};
                mt.box[0] = std::min(mt.box[0], x0);
                mt.box[1] = std::max(mt.box[1], x0);
                mt.box[2] = std::min(mt.box[2], x1);
                mt.box[3] = std::max(mt.box[3], x1);
            }
            mt.winding = orient2(mt.p[0], mt.p[1], mt.p[2]);
            mt.sign = tri_sign[t];
            mixed[g].push_back(std::move(mt));
        }
    }

    // sign(n_facet . ref_unit), where n_facet is the facet's normal in ascending vertex order.
    // n_facet is exactly parallel to the reference, so the dot is +-|n_facet| and its sign is
    // the answer; the double evaluation is trusted once it clears the rounding bound (input
    // coordinates rounded to double, then a cross product of differences of them).
    const auto facet_sign = [&](const std::array<size_t, 3>& f, size_t g) {
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

    // The coverage of mixed group g at the point q of its plane, given by its projection
    // (exact) and that projection rounded; nullopt when q lies on an edge of one of the
    // group's triangles, and so tells nothing about that one.
    const auto coverage_at = [&](const std::array<Rational, 2>& q,
                                 const std::array<double, 2>& qd,
                                 size_t g) -> std::optional<int> {
        int cover = 0;
        for (const MixedTri& mt : mixed[g]) {
            const double m0 = 1e-9 * (std::abs(mt.box[0]) + std::abs(mt.box[1])) + 1e-300;
            const double m1 = 1e-9 * (std::abs(mt.box[2]) + std::abs(mt.box[3])) + 1e-300;
            if (qd[0] < mt.box[0] - m0 || qd[0] > mt.box[1] + m0 || qd[1] < mt.box[2] - m1 ||
                qd[1] > mt.box[3] + m1) {
                continue;
            }
            bool inside = true;
            for (int j = 0; j < 3; ++j) {
                const auto& u = mt.p[j];
                const auto& v = mt.p[(j + 1) % 3];
                const int o = orient2(u, v, q);
                if (o == 0) {
                    bool between = true;
                    for (int ax = 0; ax < 2; ++ax) {
                        const Rational& lo = u[ax] < v[ax] ? u[ax] : v[ax];
                        const Rational& hi = u[ax] < v[ax] ? v[ax] : u[ax];
                        if (q[ax] < lo || q[ax] > hi) between = false;
                    }
                    if (between) return std::nullopt;
                }
                if (o != mt.winding) inside = false;
            }
            if (inside) cover += mt.sign;
        }
        return cover;
    };

    // Interior points of a facet to try, as positive integer barycentric weights: a fixed list
    // in general position, then a deterministic sequence, so a facet is not left without an
    // answer short of pathological degeneracy. The first two on no edge of the group's
    // triangles are used: the first gives the answer, the second notices a facet straddling a
    // change of coverage.
    static constexpr int fixed_weights[8][3] =
        {{1, 1, 1}, {5, 3, 2}, {2, 5, 3}, {3, 2, 5}, {7, 4, 2}, {2, 7, 4}, {4, 2, 7}, {11, 6, 5}};
    const auto weights = [](int k) -> std::array<int, 3> {
        if (k < 8) return {{fixed_weights[k][0], fixed_weights[k][1], fixed_weights[k][2]}};
        return {{1 + (k * 37) % 97, 1 + (k * 61) % 89, 1 + (k * 83) % 101}};
    };
    constexpr int max_test_points = 64;

    std::vector<int> res(facets.size(), 0);
    // Strided rather than blocked: the facets of one mixed group, the expensive ones, tend to
    // be contiguous.
    const size_t n_workers = std::max<size_t>(1, num_threads > 0 ? size_t(num_threads) : size_t(1));
    std::vector<size_t> straddling(n_workers, 0), unanswered(n_workers, 0);
    threading::parallel_for(
        threading::range(0, n_workers),
        [&](const threading::range& r) {
            for (size_t w = r.begin(); w < r.end(); ++w) {
                for (size_t i = w; i < facets.size(); i += n_workers) {
                    if (!facets_on_input[i]) continue;
                    std::array<size_t, 3> f = facets[i];
                    std::sort(f.begin(), f.end());
                    const auto [lo, hi] = groups_of(f);
                    int o = 0;
                    for (auto it = lo; it != hi; ++it) {
                        const size_t g = it->second;
                        if (g >= n_groups || group_tris[g].empty()) continue;
                        const int fs = facet_sign(f, g);
                        if (uniform[g]) {
                            o += fs;
                            continue;
                        }
                        const int ax0 = (drop_axis[g] + 1) % 3, ax1 = (drop_axis[g] + 2) % 3;
                        std::optional<int> first;
                        bool straddles = false;
                        for (int k = 0; k < max_test_points; ++k) {
                            const auto wt = weights(k);
                            const Rational sum(wt[0] + wt[1] + wt[2]);
                            std::array<Rational, 2> q;
                            std::array<double, 2> qd;
                            for (const int ax : {ax0, ax1}) {
                                const int a = ax == ax0 ? 0 : 1;
                                q[a] = (v_rational[f[0]][ax] * Rational(wt[0]) +
                                        v_rational[f[1]][ax] * Rational(wt[1]) +
                                        v_rational[f[2]][ax] * Rational(wt[2])) /
                                       sum;
                                qd[a] = q[a].to_double();
                            }
                            const auto c = coverage_at(q, qd, g);
                            if (!c.has_value()) continue;
                            const int sc = (*c > 0) - (*c < 0);
                            if (!first.has_value()) {
                                first = sc;
                                continue;
                            }
                            straddles = sc != *first;
                            break;
                        }
                        if (!first.has_value()) {
                            ++unanswered[w];
                            continue;
                        }
                        if (straddles) ++straddling[w];
                        o += fs * *first;
                    }
                    res[i] = o;
                }
            }
        },
        int(n_workers));

    n_straddling = std::accumulate(straddling.begin(), straddling.end(), size_t(0));
    const size_t n_unanswered = std::accumulate(unanswered.begin(), unanswered.end(), size_t(0));
    if (n_unanswered > 0) {
        logger().warn(
            "orientation of {} facets: every test point lies on an input edge; left 0",
            n_unanswered);
    }
    return res;
}

/**
 * Make the oriented surface consistent patch by patch, each patch keeping the orientation most
 * of its area has.
 *
 * The facets of orientation +-1 are linked across every edge exactly two of them share and no
 * other facet of nonzero orientation touches -- a manifold edge of the oriented surface -- and
 * each linked patch is oriented consistently by orient_by_majority, weighted by area: the
 * facets that disagree with their patch -- where the input itself was flipped, or a facet
 * straddled such a flip (see facet_orientations) -- are reversed. On a consistently oriented
 * input nothing changes, every link being consistent already. Facets of orientation 0 (where
 * touching solids cancel) are not part of the oriented surface, and those of larger magnitude
 * (coincident sheets) are left as computed and end a patch. fTetWild orients its tracked
 * surface the same way: from the input, then a BFS per patch.
 */
OrientationRepair orient_facet_patches(
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::array<size_t, 3>>& facets,
    std::vector<int>& orientation)
{
    assert(v_rational.size() < size_t(UINT32_MAX));
    constexpr uint32_t not_unit = UINT32_MAX;
    std::vector<uint32_t> unit; // facet of each element
    std::vector<uint32_t> elem(facets.size(), not_unit);
    size_t n_nonzero = 0;
    for (size_t i = 0; i < facets.size(); ++i) {
        if (orientation[i] == 0) continue;
        ++n_nonzero;
        if (orientation[i] == 1 || orientation[i] == -1) {
            elem[i] = uint32_t(unit.size());
            unit.push_back(uint32_t(i));
        }
    }

    // Every edge of every facet of nonzero orientation, directed by that orientation.
    struct HalfEdge
    {
        uint64_t key; // (min << 32) | max
        uint32_t elem; // not_unit for a facet of larger multiplicity
        bool ascending; // runs min -> max
    };
    std::vector<HalfEdge> half_edges;
    half_edges.reserve(3 * n_nonzero);
    for (size_t i = 0; i < facets.size(); ++i) {
        if (orientation[i] == 0) continue;
        std::array<size_t, 3> f = facets[i];
        std::sort(f.begin(), f.end());
        if (orientation[i] < 0) std::swap(f[1], f[2]);
        for (int j = 0; j < 3; ++j) {
            const uint64_t u = f[j], v = f[(j + 1) % 3];
            half_edges.push_back({(std::min(u, v) << 32) | std::max(u, v), elem[i], u < v});
        }
    }
    std::sort(half_edges.begin(), half_edges.end(), [](const HalfEdge& a, const HalfEdge& b) {
        return a.key != b.key ? a.key < b.key : a.elem < b.elem;
    });
    std::vector<OrientationLink> links;
    for (size_t s = 0; s < half_edges.size();) {
        size_t e = s + 1;
        while (e < half_edges.size() && half_edges[e].key == half_edges[s].key) ++e;
        if (e - s == 2 && half_edges[s].elem != not_unit && half_edges[s + 1].elem != not_unit) {
            links.push_back(
                {half_edges[s].elem,
                 half_edges[s + 1].elem,
                 half_edges[s].ascending != half_edges[s + 1].ascending});
        }
        s = e;
    }
    std::vector<HalfEdge>().swap(half_edges);

    std::vector<double> area(unit.size());
    for (size_t k = 0; k < unit.size(); ++k) {
        const auto& f = facets[unit[k]];
        const Vector3d a = to_double(v_rational[f[0]]);
        const Vector3d b = to_double(v_rational[f[1]]);
        const Vector3d c = to_double(v_rational[f[2]]);
        area[k] = (b - a).cross(c - a).norm();
    }

    OrientationRepair rep = orient_by_majority(unit.size(), links, area);
    for (size_t k = 0; k < unit.size(); ++k) {
        if (rep.turn[k]) orientation[unit[k]] = -orientation[unit[k]];
    }
    return rep;
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
    std::vector<int8_t>* tet_face_orientation)
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

    // Per facet: the input's orientation on it, against the facet's ascending vertex order --
    // see facet_orientations -- then made consistent patch by patch.
    std::vector<int> facet_orientation;
    if (tet_face_orientation != nullptr) {
        logger().info("Orienting the tracked surface...");
        size_t n_straddling = 0;
        facet_orientation = facet_orientations(
            tri_vrt_coord,
            triangle_indices,
            vr_tri_group,
            vr_tri_provenance.size(),
            v_rational,
            polygon_faces,
            polygon_faces_on_input,
            groups_of,
            opts.num_threads,
            n_straddling);
        const OrientationRepair rep =
            orient_facet_patches(v_rational, polygon_faces, facet_orientation);
        if (n_straddling > 0 || rep.n_turned > 0 || rep.n_non_orientable > 0) {
            logger().info(
                "tracked surface: the input is not consistently oriented -- {} facets straddle a "
                "flip, {} reversed to agree with their patch, {} non-orientable patches left as "
                "they are",
                n_straddling,
                rep.n_turned,
                rep.n_non_orientable);
        }
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
        std::array<int8_t, 4> tet_face_orient{{0, 0, 0, 0}};
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
            if (tet_face_orientation != nullptr) {
                tet_face_orient[local_f_idx] = int8_t(std::clamp(facet_orientation[f], -127, 127));
            }
        }

        for (int k = 0; k < 4; k++) {
            tet_face_on_input_surface.push_back(tet_face_on_input[k]);
            if (tet_face_orientation != nullptr) {
                tet_face_orientation->push_back(tet_face_orient[k]);
            }
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
            const std::array<int8_t, 4> opp{
                {o[4 * i + 0], o[4 * i + 1], o[4 * i + 2], o[4 * i + 3]}};
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
