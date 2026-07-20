#include "init_from_delaunay.hpp"

#include <VolumeRemesher/2d/embed2d.h>
#include <VolumeRemesher/numerics.h>

#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

#include <array>
#include <bitset>
#include <cstdint>
#include <set>

namespace wmtk::components::triwild {

namespace {

// The remesher hands back exact coordinates; which concrete bignum type depends on
// how VolumeRemesher was configured (see cmake/recipes/volumemesher.cmake). Go
// through wmtk::Rational -- the same conversion the 3D insertion uses -- so both
// backends are handled, then round once to double.
double to_double(const vol_rem::bigrational& r)
{
    Rational q;
#ifdef USE_GNU_GMP_CLASSES
    q.init(r.get_mpq_t());
#else
    q.init_from_bin(r.get_str());
#endif
    return q.to_double();
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
    {
        std::vector<Vector2d> V_envelope;
        std::vector<Vector2i> E_envelope;
        for (int i = 0; i < E.rows(); i++) {
            E_envelope.push_back(Vector2i(E(i, 0), E(i, 1)));
        }
        for (int i = 0; i < V.rows(); i++) {
            V_envelope.push_back(V.row(i));
        }
        envelope.init(V_envelope, E_envelope, 0);
    }

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

} // namespace

void init_from_delaunay_box_mesh(
    const MatrixXd& V,
    const MatrixXi& E,
    MatrixXd& V_out,
    MatrixXi& F_out,
    MatrixXi& E_out)
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
    std::vector<vol_rem::bigrational> vertices;
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

    const int nv = int(vertices.size() / 2);
    V_out.resize(nv, 2);
    for (int v = 0; v < nv; ++v) {
        V_out(v, 0) = to_double(vertices[2 * v + 0]);
        V_out(v, 1) = to_double(vertices[2 * v + 1]);
    }

    F_out.resize(tris.size(), 3);
    for (size_t t = 0; t < tris.size(); ++t) {
        F_out(t, 0) = int(tris[t][0]);
        F_out(t, 1) = int(tris[t][1]);
        F_out(t, 2) = int(tris[t][2]);
    }

    // The constrained edges of the output are the union of the per-segment edge
    // lists. Overlapping input segments share output edges, so deduplicate; a
    // std::set keyed on the sorted vertex pair also fixes the row order, which
    // keeps the result reproducible.
    std::set<std::pair<int, int>> constrained_edges;
    for (const auto& seg : segment_provenance) {
        for (const auto& e : seg) {
            int a = int(e[1]);
            int b = int(e[2]);
            if (a > b) {
                std::swap(a, b);
            }
            constrained_edges.insert({a, b});
        }
    }

    E_out.resize(constrained_edges.size(), 2);
    {
        int idx = 0;
        for (const auto& edge : constrained_edges) {
            E_out(idx, 0) = edge.first;
            E_out(idx, 1) = edge.second;
            ++idx;
        }
    }

    logger().info(
        "2D arrangement: #V = {}, #F = {}, #E_constrained = {}",
        V_out.rows(),
        F_out.rows(),
        E_out.rows());
}

void init_from_paths(
    const std::vector<std::string>& input_paths,
    MatrixXd& V_out,
    MatrixXi& F_out,
    MatrixXi& E_out)
{
    std::vector<MatrixXd> Vs;
    std::vector<MatrixXi> Es;
    // read input edge meshes
    for (const std::string& path : input_paths) {
        MatrixXd V;
        MatrixXi E;
        io::read_edge_mesh(path, V, E);
        logger().info("Read edge mesh {}: #V = {}, #E = {}", path, V.rows(), E.rows());
        V = V.block(0, 0, V.rows(), 2).eval(); // only keep x, y
        Vs.push_back(V);
        Es.push_back(E);
    }
    if (Vs.size() != Es.size()) {
        log_and_throw_error("Vs and Es size mismatch!");
    }

    // generate Delaunay triangulation of the input vertices as the initial mesh
    {
        size_t num_vertices = 0;
        MatrixXd V_all;
        MatrixXi E_all;
        std::vector<Eigen::Vector2d> V_vec;
        std::vector<Eigen::Vector2i> E_vec;
        for (size_t i = 0; i < Vs.size(); i++) {
            for (int j = 0; j < Vs[i].rows(); j++) {
                V_vec.push_back(Vs[i].row(j));
            }
            MatrixXi E = Es[i];
            E.array() += num_vertices; // offset the vertex indices
            for (int j = 0; j < E.rows(); j++) {
                E_vec.push_back(E.row(j));
            }
            num_vertices += Vs[i].rows();
        }

        V_all.resize(V_vec.size(), 2);
        for (int i = 0; i < V_vec.size(); i++) {
            V_all.row(i) = V_vec[i];
        }
        E_all.resize(E_vec.size(), 2);
        for (int i = 0; i < E_vec.size(); i++) {
            E_all.row(i) = E_vec[i];
        }
        init_from_delaunay_box_mesh(V_all, E_all, V_out, F_out, E_out);
    }

    // logger().info("CDT mesh: #V = {}, #F = {}, #E = {}", V_out.rows(), F_out.rows(),
    // E_out.rows());
}

} // namespace wmtk::components::triwild