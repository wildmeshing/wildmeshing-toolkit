#include "TriWild.h"
#include <fastenvelope/FastEnvelope.h>
#include <igl/predicates/predicates.h>
#include <igl/write_triangle_mesh.h>
#include <tbb/concurrent_vector.h>
#include <wmtk/utils/AMIPS2D.h>
#include <Eigen/Core>
#include <wmtk/utils/TupleUtils.hpp>
using namespace wmtk;
namespace triwild {
auto avg_edge_len = [](auto& m) {
    double avg_len = 0.0;
    auto edges = m.get_edges();
    for (auto& e : edges) avg_len += std::sqrt(m.get_length2(e));
    return avg_len / edges.size();
};

bool TriWild::invariants(const std::vector<Tuple>& new_tris)
{
    if (m_has_envelope) {
        for (auto& t : new_tris) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = oriented_tri_vertices(t);
            for (auto j = 0; j < 3; j++) {
                tris[j] << vertex_attrs[vs[j].vid(*this)].pos(0),
                    vertex_attrs[vs[j].vid(*this)].pos(1), 0.0;
            }
            if (m_envelope.is_outside(tris)) return false;
        }
    }
    return true;
}
std::vector<TriMesh::Tuple> TriWild::new_edges_after(const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(*this), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

void TriWild::create_mesh(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    double eps,
    bool bnd_freeze)
{
    std::vector<Eigen::Vector3d> V_env;
    V_env.resize(V.rows());
    std::vector<Eigen::Vector3i> F_env;
    F_env.resize(F.rows());
    // Register attributes
    p_vertex_attrs = &vertex_attrs;

    // Convert from eigen to internal representation (TODO: move to utils and remove it from all
    // app)
    std::vector<std::array<size_t, 3>> tri(F.rows());

    for (int i = 0; i < F.rows(); i++) {
        F_env[i] << (size_t)F(i, 0), (size_t)F(i, 1), (size_t)F(i, 2);
        for (int j = 0; j < 3; j++) {
            tri[i][j] = (size_t)F(i, j);
        }
    }
    // Initialize the trimesh class which handles connectivity
    wmtk::TriMesh::create_mesh(V.rows(), tri);
    // Save the vertex position in the vertex attributes
    for (unsigned i = 0; i < V.rows(); ++i) {
        vertex_attrs[i].pos << V.row(i)[0], V.row(i)[1];
        V_env[i] << V.row(i)[0], V.row(i)[1], 0.0;
    }
    for (auto tri : this->get_faces()) {
        assert(!is_inverted(tri));
    }

    // mark boundary vertices as fixed
    // but this is not indiscriminatively fixed for all operations
    // only swap will always be reject for boundary edges
    // other operations are conditioned on whether m_bnd_freeze is turned on
    for (auto v : this->get_vertices()) {
        vertex_attrs[v.vid(*this)].fixed = is_boundary_vertex(v);
    }

    if (eps > 0) {
        m_envelope.use_exact = true;
        m_envelope.init(V_env, F_env, eps);
        m_has_envelope = true;
    } else if (bnd_freeze) {
        m_bnd_freeze = bnd_freeze;
    }
}

void TriWild::export_mesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V = Eigen::MatrixXd::Zero(vert_capacity(), 2);
    for (auto& t : get_vertices()) {
        auto i = t.vid(*this);
        V.row(i) = vertex_attrs[i].pos;
    }

    F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
    for (auto& t : get_faces()) {
        auto i = t.fid(*this);
        auto vs = oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            F(i, j) = vs[j].vid(*this);
        }
    }
}

void TriWild::write_obj(const std::string& path)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    export_mesh(V, F);

    Eigen::MatrixXd V3 = Eigen::MatrixXd::Zero(V.rows(), 3);
    V3.leftCols(2) = V;

    igl::writeOBJ(path, V3, F);
}

double TriWild::get_length2(const Tuple& t) const
{
    auto& m = *this;
    auto& v1 = t;
    auto v2 = t.switch_vertex(m);
    double length = (m.vertex_attrs[v1.vid(m)].pos - m.vertex_attrs[v2.vid(m)].pos).squaredNorm();
    return length;
}

double TriWild::get_quality(const Tuple& loc) const
{
    // Global ids of the vertices of the triangle
    auto its = oriented_tri_vids(loc);

    // Temporary variable to store the stacked coordinates of the triangle
    std::array<double, 6> T;
    auto energy = -1.;
    for (auto k = 0; k < 3; k++)
        for (auto j = 0; j < 2; j++) T[k * 2 + j] = vertex_attrs[its[k]].pos[j];

    // Energy evaluation
    energy = wmtk::AMIPS2D_energy(T);

    // Filter for numerical issues
    if (std::isinf(energy) || std::isnan(energy)) return MAX_ENERGY;

    return energy;
}

Eigen::VectorXd TriWild::get_quality_all_triangles()
{
    // Use a concurrent vector as for_each_face is parallel
    tbb::concurrent_vector<double> quality;
    quality.reserve(vertex_attrs.size());

    // Evaluate quality in parallel
    for_each_face([&](auto& f) { quality.push_back(get_quality(f)); });

    // Copy back in a VectorXd
    Eigen::VectorXd ret(quality.size());
    for (unsigned i = 0; i < quality.size(); ++i) ret[i] = quality[i];
    return ret;
}

bool TriWild::is_inverted(const Tuple& loc) const
{
    // Get the vertices ids
    auto vs = oriented_tri_vertices(loc);

    igl::predicates::exactinit();

    // Use igl for checking orientation
    auto res = igl::predicates::orient2d(
        vertex_attrs[vs[0].vid(*this)].pos,
        vertex_attrs[vs[1].vid(*this)].pos,
        vertex_attrs[vs[2].vid(*this)].pos);

    // The element is inverted if it not positive (i.e. it is negative or it is degenerate)
    return (res != igl::predicates::Orientation::POSITIVE);
}

void TriWild::mesh_improvement(int max_its)
{
    double avg_len = 0.0;
    double pre_avg_len = 0.0;
    double pre_max_energy = -1.0;
    wmtk::logger().info("target len {}", m_target_l);
    for (int it = 0; it < max_its; it++) {
        ///ops
        wmtk::logger().info("\n========it {}========", it);

        ///energy check
        wmtk::logger().info("max energy {} stop energy {}", m_max_energy, m_stop_energy);
        collapse_all_edges();
        // write_obj("after_collapse_" + std::to_string(it) + ".obj");

        split_all_edges();
        // write_obj("after_split_" + std::to_string(it) + ".obj");

        swap_all_edges();
        // write_obj("after_swap_" + std::to_string(it) + ".obj");


        smooth_all_vertices();
        // write_obj("after_smooth_" + std::to_string(it) + ".obj");

        wmtk::logger().info(
            "++++++++v {} t {} max energy {}++++++++",
            vert_capacity(),
            tri_capacity(),
            m_max_energy);

        avg_len = avg_edge_len(*this);
        if (m_target_l <= 0 && m_max_energy < m_stop_energy) {
            break;
        }

        if (m_target_l > 0 && (avg_len - m_target_l) * (avg_len - m_target_l) < 1e-4) {
            wmtk::logger().info(
                "doesn't improve anymore. Stopping improvement.\n {} itr finished, max energy {}",
                it,
                m_max_energy);
            break;
        }
        if (it > 0 &&
            (m_target_l <= 0 &&
             (pre_max_energy - m_stop_energy) * (pre_max_energy - m_stop_energy) < 1e-2)) {
            wmtk::logger().info(
                "doesn't improve anymore. Stopping improvement.\n {} itr finished, max energy {}",
                it,
                m_max_energy);
            break;
        }
        wmtk::logger().info("current length {}", avg_len);
        pre_avg_len = avg_len;
        pre_max_energy = m_max_energy;
    }

    wmtk::logger().info("/////final: max energy {} , avg len {} ", m_max_energy, avg_len);
    consolidate_mesh();
}

} // namespace triwild