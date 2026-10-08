#include "EmbedSegments.hpp"

#include <VolumeRemesher/2d/embed2d.h>

#include <algorithm>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/orient_by_majority.hpp>

#include <array>
#include <bitset>
#include <cstdint>
#include <map>
#include <set>

namespace wmtk::utils {

namespace {

// The remesher hands back exact coordinates; which concrete bignum type depends on
// how VolumeRemesher was configured (see cmake/recipes/volumeremesher.cmake). Go
// through wmtk::Rational -- the same conversion the 3D insertion uses -- so both
// backends are handled.
Rational bigrational_to_rational(const NFG::bigrational& r)
{
    Rational q;
#ifdef USE_GNU_GMP_CLASSES
    q.init(r.get_mpq_t());
#else
    q.init_from_bin(r.get_str());
#endif
    return q;
}

/**
 * @brief Append a voxel lattice covering the input's bounding box, grown by a fifteenth of
 * its diagonal, to the flat coordinate array `coords`.
 *
 * The arrangement triangulates every point it is handed, whether or not a segment
 * references it, so these act purely as background points. Lattice points closer than half
 * a voxel to an input segment are skipped: they would only crowd the constraints, which the
 * arrangement is about to refine anyway.
 *
 * Seeding this way is what the geogram CDT path did before the arrangement replaced it.
 * Without it the initial mesh is just the arrangement of the input curves -- valid, but
 * sparse and badly shaped away from the input, which costs mesh quality and makes the
 * optimization phase work harder to recover.
 */
void append_background_grid(const MatrixXd& V, const MatrixXi& E, std::vector<double>& coords)
{
    Vector2d box_min = V.colwise().minCoeff();
    Vector2d box_max = V.colwise().maxCoeff();

    const double diagonal_length = (box_max - box_min).norm();
    const double delta = diagonal_length / 15.0;
    box_min -= Vector2d(delta, delta);
    box_max += Vector2d(delta, delta);

    const auto push = [&coords](double x, double y) {
        coords.push_back(x);
        coords.push_back(y);
    };

    // corners of the domain
    for (int i = 0; i < 4; i++) {
        const std::bitset<2> a(i);
        push(a.test(0) ? box_max[0] : box_min[0], a.test(1) ? box_max[1] : box_min[1]);
    }

    const double voxel_resolution = diagonal_length / 20.0;
    std::array<int, 2> N; // number of grid points per dimension
    std::array<double, 2> h; // distance between grid points per dimension
    for (int i = 0; i < 2; i++) {
        const double D = box_max[i] - box_min[i];
        N[i] = (D / voxel_resolution) + 1;
        h[i] = D / N[i];
    }

    std::array<std::vector<double>, 2> ds;
    for (int i = 0; i < 2; i++) {
        ds[i].push_back(box_min[i]);
        for (int j = 0; j < N[i] - 1; j++) {
            ds[i].push_back(box_min[i] + h[i] * (j + 1));
        }
        ds[i].push_back(box_max[i]);
    }

    SampleEnvelope envelope;
    envelope.init(V, E, 0);

    const double min_dis = voxel_resolution * voxel_resolution / 4;
    for (size_t i = 0; i < ds[0].size(); i++) {
        for (size_t j = 0; j < ds[1].size(); j++) {
            if ((i == 0 || i == ds[0].size() - 1) && (j == 0 || j == ds[1].size() - 1)) {
                continue; // the four corners went in above
            }
            const Vector2d p(ds[0][i], ds[1][j]);

            Eigen::Vector2d n;
            if (envelope.nearest_point(p, n) < min_dis) {
                continue; // too close to an input segment
            }
            push(ds[0][i], ds[1][j]);
        }
    }
}

/**
 * Make the oriented curves consistent patch by patch, each keeping the direction most of its
 * length has -- the 2D counterpart of orient_facet_patches in EmbedTriangles.cpp, which see.
 * The edges of orientation +-1 are linked at every vertex exactly two of them share and no
 * other edge of nonzero orientation touches.
 */
void orient_curve_patches(const MatrixXd& V_out, const MatrixXi& E_out, std::vector<int>& o)
{
    constexpr uint32_t not_unit = UINT32_MAX;
    std::vector<uint32_t> unit; // edge of each element
    std::vector<uint32_t> elem(o.size(), not_unit);
    struct End
    {
        uint32_t v;
        uint32_t elem; // not_unit for an edge of larger multiplicity
        bool leaves; // the edge's direction leaves v
    };
    std::vector<End> ends;
    for (size_t i = 0; i < o.size(); ++i) {
        if (o[i] == 0) continue;
        if (o[i] == 1 || o[i] == -1) {
            elem[i] = uint32_t(unit.size());
            unit.push_back(uint32_t(i));
        }
        // E_out rows are (min, max) and o is measured along them.
        ends.push_back({uint32_t(E_out(i, 0)), elem[i], o[i] > 0});
        ends.push_back({uint32_t(E_out(i, 1)), elem[i], o[i] < 0});
    }
    std::sort(ends.begin(), ends.end(), [](const End& a, const End& b) {
        return a.v != b.v ? a.v < b.v : a.elem < b.elem;
    });
    std::vector<OrientationLink> links;
    for (size_t s = 0; s < ends.size();) {
        size_t e = s + 1;
        while (e < ends.size() && ends[e].v == ends[s].v) ++e;
        if (e - s == 2 && ends[s].elem != not_unit && ends[s + 1].elem != not_unit) {
            links.push_back({ends[s].elem, ends[s + 1].elem, ends[s].leaves != ends[s + 1].leaves});
        }
        s = e;
    }
    std::vector<double> length(unit.size());
    for (size_t k = 0; k < unit.size(); ++k) {
        length[k] = (V_out.row(E_out(unit[k], 1)) - V_out.row(E_out(unit[k], 0))).norm();
    }
    const OrientationRepair rep = orient_by_majority(unit.size(), links, length);
    for (size_t k = 0; k < unit.size(); ++k) {
        if (rep.turn[k]) o[unit[k]] = -o[unit[k]];
    }
    if (rep.n_turned > 0 || rep.n_non_orientable > 0) {
        logger().info(
            "tracked curves: the input is not consistently oriented -- {} edges reversed to agree "
            "with their curve, {} non-orientable patches left as they are",
            rep.n_turned,
            rep.n_non_orientable);
    }
}

} // namespace

void embed_segments(
    const MatrixXd& V,
    const MatrixXi& E,
    MatrixXd& V_out,
    std::vector<Vector2r>& V_rational,
    MatrixXi& F_out,
    MatrixXi& E_out,
    std::vector<std::vector<int>>* E_out_sources,
    std::vector<int>* E_out_orientation)
{
    assert(V.cols() == 2);
    assert(E.cols() == 2);

    // Flatten the input into the segment soup the remesher expects: the points as
    // x0,y0,x1,y1,... and the segments as endpoint index pairs into that array. The input
    // points go first so the segment indices below can index them directly, which also
    // leaves room to append background points afterwards without disturbing them.
    std::vector<double> seg_vrt_coords(2 * V.rows());
    for (int i = 0; i < V.rows(); ++i) {
        seg_vrt_coords[2 * i + 0] = V(i, 0);
        seg_vrt_coords[2 * i + 1] = V(i, 1);
    }

    std::vector<uint32_t> segment_indexes(2 * E.rows());
    for (int i = 0; i < E.rows(); ++i) {
        for (int k = 0; k < 2; ++k) {
            if (E(i, k) < 0 || E(i, k) >= V.rows()) {
                log_and_throw_error("Edge index out of bounds at index {}: {}", i, E.row(i));
            }
            segment_indexes[2 * i + k] = uint32_t(E(i, k));
        }
    }

    // Seed the triangulation with background points. Appended after the input points, and
    // referenced by no segment, so the indices above stay valid.
    append_background_grid(V, E, seg_vrt_coords);

    // Exact arrangement of the segment soup, as a triangulation covering the input
    // bounding box grown by 10%. Segments may cross, overlap or be duplicated; the
    // remesher resolves all of that and reports, per input segment, the output
    // triangle edges that tile it.
    std::vector<NFG::bigrational> vertices;
    std::vector<std::array<uint32_t, 3>> tris;
    std::vector<std::vector<std::array<uint32_t, 3>>> segment_provenance;
    std::vector<std::array<uint32_t, 2>> point_provenance;

    if (!vol_rem::embed_seg_in_tri_mesh(
            seg_vrt_coords,
            segment_indexes,
            vertices,
            tris,
            segment_provenance,
            point_provenance,
            false)) {
        log_and_throw_error("2D arrangement of the input segments failed");
    }
    assert(vertices.size() % 2 == 0);

    // Keep the exact coordinates and hand back the rounding alongside them. Rounding here
    // and throwing the rationals away is what used to make distinct arrangement vertices
    // collide on the same double -- see the header.
    const int nv = int(vertices.size() / 2);
    V_rational.resize(nv);
    V_out.resize(nv, 2);
    size_t n_indirect = 0;
    for (int v = 0; v < nv; ++v) {
        V_rational[v][0] = bigrational_to_rational(vertices[2 * v + 0]);
        V_rational[v][1] = bigrational_to_rational(vertices[2 * v + 1]);
        V_out(v, 0) = V_rational[v][0].to_double();
        V_out(v, 1) = V_rational[v][1].to_double();
        if (Rational(V_out(v, 0)) != V_rational[v][0] ||
            Rational(V_out(v, 1)) != V_rational[v][1]) {
            ++n_indirect;
        }
    }
    // As in embed_triangles_in_tets: free the remesher's numbers, then the memory NFG's
    // bignatural pool grew into for them, which it never returns on its own (52 MB of a 245 MB
    // peak on Thingi10K 193153).
    std::vector<NFG::bigrational>().swap(vertices);
    NFG::bignatural::trimMemoryPool();

    F_out.resize(tris.size(), 3);
    for (size_t t = 0; t < tris.size(); ++t) {
        F_out(t, 0) = int(tris[t][0]);
        F_out(t, 1) = int(tris[t][1]);
        F_out(t, 2) = int(tris[t][2]);
    }

    // The constrained edges of the output are the union of the per-segment edge
    // lists. Overlapping input segments share output edges, so deduplicate; a
    // std::map keyed on the sorted vertex pair also fixes the row order, which
    // keeps the result reproducible. The value is which input segments produced
    // the edge -- the provenance the caller would otherwise have to guess back
    // geometrically, and cannot where two inputs overlap.
    //
    // The remesher orders each segment's provenance from its first endpoint to its second, with
    // {triangle, v0, v1} and v0 the nearer the first endpoint: v0 -> v1 is the input's direction,
    // and the sort below would discard it, so count it first.
    //
    // An input segment repeated -- the same two endpoint positions, either way round -- is one
    // sheet, as a coplanar group of triangles is in 3D (see embed_triangles_in_tets): its copies
    // count once, by the sign of their net direction, carried by the first of them. Distinct
    // segments overlapping still add.
    std::vector<int> seg_weight(E.rows(), 0);
    if (E_out_orientation != nullptr) {
        using Key = std::array<double, 4>; // the endpoints, lexicographically ascending
        std::map<Key, std::pair<int, int>> copies; // -> (first copy, net direction along key)
        std::vector<int> along(E.rows());
        for (int i = 0; i < E.rows(); ++i) {
            const std::array<double, 2> p{{V(E(i, 0), 0), V(E(i, 0), 1)}};
            const std::array<double, 2> q{{V(E(i, 1), 0), V(E(i, 1), 1)}};
            along[i] = p < q ? 1 : -1;
            const Key k = p < q ? Key{{p[0], p[1], q[0], q[1]}} : Key{{q[0], q[1], p[0], p[1]}};
            copies.try_emplace(k, i, 0).first->second.second += along[i];
        }
        for (const auto& [k, c] : copies) {
            const int net = c.second;
            seg_weight[c.first] = ((net > 0) - (net < 0)) * along[c.first];
        }
    }
    std::map<std::pair<int, int>, std::vector<int>> constrained_edges;
    std::map<std::pair<int, int>, int> orientation; // along (min, max)
    for (size_t s = 0; s < segment_provenance.size(); ++s) {
        for (const auto& e : segment_provenance[s]) {
            int a = int(e[1]);
            int b = int(e[2]);
            const int dir = a < b ? 1 : -1;
            if (a > b) {
                std::swap(a, b);
            }
            orientation[{a, b}] += dir * seg_weight[s];
            auto& src = constrained_edges[{a, b}];
            if (src.empty() || src.back() != int(s)) {
                src.push_back(int(s)); // s ascends, so this keeps it sorted and unique
            }
        }
    }

    E_out.resize(constrained_edges.size(), 2);
    if (E_out_sources != nullptr) {
        E_out_sources->assign(constrained_edges.size(), {});
    }
    if (E_out_orientation != nullptr) {
        E_out_orientation->assign(constrained_edges.size(), 0);
    }
    {
        int idx = 0;
        for (const auto& [edge, src] : constrained_edges) {
            E_out(idx, 0) = edge.first;
            E_out(idx, 1) = edge.second;
            if (E_out_sources != nullptr) {
                (*E_out_sources)[idx] = src;
            }
            if (E_out_orientation != nullptr) {
                (*E_out_orientation)[idx] = orientation[edge];
            }
            ++idx;
        }
    }
    if (E_out_orientation != nullptr) {
        orient_curve_patches(V_out, E_out, *E_out_orientation);
    }

    logger().info(
        "2D arrangement: #V = {}, #F = {}, #E_constrained = {} ({} vertices have no exact "
        "double representation)",
        V_out.rows(),
        F_out.rows(),
        E_out.rows(),
        n_indirect);
}

void read_input_curves(
    const std::vector<std::string>& input_paths,
    double remove_duplicate_eps,
    MatrixXd& V_all,
    MatrixXi& E_all,
    std::vector<MatrixXd>& Vs_out,
    std::vector<MatrixXi>& Es_out)
{
    Vs_out.clear();
    Es_out.clear();

    // read input edge meshes
    for (const std::string& path : input_paths) {
        MatrixXd V;
        MatrixXi E;
        io::read_edge_mesh(path, V, E, remove_duplicate_eps);
        logger().info("Read edge mesh {}: #V = {}, #E = {}", path, V.rows(), E.rows());
        V = V.block(0, 0, V.rows(), 2).eval(); // only keep x, y
        Vs_out.push_back(V);
        Es_out.push_back(E);
    }

    // Concatenate them into one segment network. The simplification and the arrangement
    // both work on the union so that curves sharing a boundary stay coincident; the
    // per-input copies are kept for the winding-number tags.
    size_t num_vertices = 0;
    std::vector<Eigen::Vector2d> V_vec;
    std::vector<Eigen::Vector2i> E_vec;
    for (size_t i = 0; i < Vs_out.size(); i++) {
        for (int j = 0; j < Vs_out[i].rows(); j++) {
            V_vec.push_back(Vs_out[i].row(j));
        }
        MatrixXi E = Es_out[i];
        E.array() += num_vertices; // offset the vertex indices
        for (int j = 0; j < E.rows(); j++) {
            E_vec.push_back(E.row(j));
        }
        num_vertices += Vs_out[i].rows();
    }

    V_all.resize(V_vec.size(), 2);
    for (size_t i = 0; i < V_vec.size(); i++) {
        V_all.row(i) = V_vec[i];
    }
    E_all.resize(E_vec.size(), 2);
    for (size_t i = 0; i < E_vec.size(); i++) {
        E_all.row(i) = E_vec[i];
    }
}

} // namespace wmtk::utils