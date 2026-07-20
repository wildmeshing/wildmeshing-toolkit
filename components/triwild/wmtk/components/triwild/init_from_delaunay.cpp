#include "init_from_delaunay.hpp"

#include <VolumeRemesher/2d/embed2d.h>
#include <VolumeRemesher/numerics.h>

#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

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
    // x0,y0,x1,y1,... and the segments as endpoint index pairs into that array.
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