#include "AdaptiveTessellation.h"
using namespace adaptive_tessellation;
using namespace wmtk;

// get the energy defined by edge_length_energy over each face of the mesh
// assuming the vert_capacity() == get_vertices.size()
double AdaptiveTessellation::get_mesh_energy(const Eigen::VectorXd& v_flat)
{
    double total_energy = 0;
    Eigen::MatrixXd energy_matrix;
    energy_matrix.resize(get_faces().size(), 2);
    for (auto& face : get_faces()) {
        // wmtk::logger().info("getting energy on {} ", f_cnt++);
        auto verts = oriented_tri_vertices(face);
        Eigen::Matrix3d v_matrix;
        v_matrix.setZero(3, 3);
        for (int i = 0; i < 3; i++) {
            const auto& vert = verts[i];
            if (is_boundary_vertex(vert) && mesh_parameters.m_boundary_parameter) {
                auto uv = mesh_parameters.m_boundary.t_to_uv(
                    vertex_attrs[vert.vid(*this)].curve_id,
                    v_flat[vert.vid(*this) * 2]);
                v_matrix.row(i) = mesh_parameters.m_project_to_3d(uv(0), uv(1));
            } else {
                auto u = v_flat[vert.vid(*this) * 2];
                auto v = v_flat[vert.vid(*this) * 2 + 1];
                v_matrix.row(i) = mesh_parameters.m_project_to_3d(u, v);
            }
        }

        assert(mesh_parameters.m_target_l != 0);
        double tri_energy = 0.;
        auto BA = v_matrix.row(1) - v_matrix.row(0);
        auto CA = v_matrix.row(2) - v_matrix.row(0);
        auto BC = v_matrix.row(1) - v_matrix.row(2);
        tri_energy += pow(BA.squaredNorm() - pow(mesh_parameters.m_target_l, 2), 2);
        tri_energy += pow(BC.squaredNorm() - pow(mesh_parameters.m_target_l, 2), 2);
        tri_energy += pow(CA.squaredNorm() - pow(mesh_parameters.m_target_l, 2), 2);

        energy_matrix(face.fid(*this), 0) = tri_energy;
        double area = (BA.cross(CA)).squaredNorm();
        double A_hat = 0.5 * (std::sqrt(3) / 2) * 0.5 *
                       pow(mesh_parameters.m_target_l, 2); // this is arbitrary now
        assert(A_hat > 0);
        if (area <= 0) {
            tri_energy += std::numeric_limits<double>::infinity();
        }
        if (area < A_hat) {
            assert((area / A_hat) < 1.0);
            tri_energy += -(area - A_hat) * (area - A_hat) * log(area / A_hat);
        }
        total_energy += tri_energy;

        energy_matrix(face.fid(*this), 1) = tri_energy;
    }
    // igl::writeDMAT("mesh_energy.dmat", energy_matrix);

    return total_energy;
}
/// debugging
void AdaptiveTessellation::gradient_debug(int max_its)
{
    split_all_edges();
    assert(invariants(get_faces()));
    consolidate_mesh();
    write_obj("before_smooth.obj");
    double old_average = 1e-4;
    for (int it = 0; it < max_its; it++) {
        wmtk::logger().info("\n========it {}========", it);
        wmtk::logger().info("current length {}", avg_edge_len());

        smooth_all_vertices();
        assert(invariants(get_faces()));
        consolidate_mesh();
        write_displaced_obj(
            "smooth_" + std::to_string(it) + ".obj",
            mesh_parameters.m_displacement);

        wmtk::logger().info(
            "++++++++v {} t {} avg gradient {}++++++++",
            vert_capacity(),
            tri_capacity(),
            mesh_parameters.m_gradient / vert_capacity());
        auto avg_grad = (mesh_parameters.m_gradient / vert_capacity()).stableNorm();

        mesh_parameters.js_log["iteration_" + std::to_string(it)]["avg_grad"] = avg_grad;
        Eigen::VectorXd v_flat;
        flatten_dofs(v_flat);
        mesh_parameters.js_log["iteration_" + std::to_string(it)]["energy"] =
            get_mesh_energy(v_flat);
        if (abs(avg_grad - old_average) < 1e-5) break;
        old_average = avg_grad;
        mesh_parameters.m_gradient = Eigen::Vector2d(0., 0.);
    }
}
