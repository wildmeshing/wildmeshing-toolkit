#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <igl/write_triangle_mesh.h>
#include "AdaptiveTessellation.h"
using namespace adaptive_tessellation;
using namespace wmtk;


//// preprocess the mesh for remeshing
// similar to mesh_processing. for debugging purpose
// using distance integral error not quadrics
void AdaptiveTessellation::mesh_preprocessing(
    const std::filesystem::path& input_mesh_path,
    const std::filesystem::path& displaced_image_path)
{
    Eigen::MatrixXd CN, FN;
    Eigen::MatrixXd V, VT;
    Eigen::MatrixXi F, FT;
    // igl::read_triangle_mesh(input_mesh_path.string(), input_V_, input_F_);
    igl::readOBJ(input_mesh_path.string(), V, VT, CN, F, FT, FN);

    wmtk::logger().info("///// #v : {} {}", VT.rows(), VT.cols());
    wmtk::logger().info("///// #f : {} {}", FT.rows(), FT.cols());
    wmtk::TriMesh m_3d;
    std::vector<std::array<size_t, 3>> tris;
    for (auto f = 0; f < F.rows(); f++) {
        std::array<size_t, 3> tri = {(size_t)F(f, 0), (size_t)F(f, 1), (size_t)F(f, 2)};
        tris.emplace_back(tri);
    }
    m_3d.create_mesh(V.rows(), tris);
    create_mesh(VT, FT);
    // set up seam edges and seam vertex coloring
    Eigen::MatrixXi E0, E1;
    std::tie(E0, E1) = seam_edges_set_up(V, F, m_3d, VT, FT);
    set_seam_vertex_coloring(V, F, m_3d, VT, FT);
    assert(E0.rows() == E1.rows());
    // construct the boundary map for boundary parametrization
    mesh_parameters.m_boundary.construct_boundaries(VT, FT, E0, E1);
    // mark boundary vertices as boundary_vertex
    // but this is not indiscriminatively rejected for all operations
    // other operations are conditioned on whether m_bnd_freeze is turned on
    // also obtain the boudnary parametrizatin t for each vertex
    // for now keep the per vertex curve-id. but this is now a edge property
    for (auto v : this->get_vertices()) {
        if (is_boundary_vertex(v)) {
            vertex_attrs[v.vid(*this)].boundary_vertex = is_boundary_vertex(v);
            set_feature(v);
            // one vertex can have more than one curve-id.
            // current curve-id ofr vertex is arbitrarily picked among them
            std::tie(vertex_attrs[v.vid(*this)].curve_id, vertex_attrs[v.vid(*this)].t) =
                mesh_parameters.m_boundary.uv_to_t(vertex_attrs[v.vid(*this)].pos);
        }
    }
    // after the boundary is constructed, set the start and end of each curve to be fixed
    set_fixed();
    // assign curve-id to each edge using the curve-it assigned for each vertex
    assign_edge_curveid();
    // cache the initial accuracy error per triangle
    std::array<wmtk::Image, 3> displaced = wmtk::load_rgb_image(displaced_image_path);
    std::vector<std::array<float, 6>> uv_triangles(tri_capacity());
    std::vector<TriMesh::Tuple> tris_tuples = get_faces();
    for (int i = 0; i < tris_tuples.size(); i++) {
        auto oriented_vids = oriented_tri_vids(tris_tuples[i]);
        for (int j = 0; j < 3; j++) {
            uv_triangles[tris_tuples[i].fid(*this)][2 * j + 0] =
                vertex_attrs[oriented_vids[j]].pos[0];
            uv_triangles[tris_tuples[i].fid(*this)][2 * j + 1] =
                vertex_attrs[oriented_vids[j]].pos[1];
        }
    }
    std::vector<float> computed_errors(tri_capacity());
    m_texture_integral = wmtk::TextureIntegral(std::move(displaced));
    m_texture_integral.get_error_per_triangle(uv_triangles, computed_errors);
    set_faces_cached_distance_integral(tris_tuples, computed_errors);
}
// get the energy defined by edge_length_energy over each face of the mesh
// assuming the vert_capacity() == get_vertices.size()
double AdaptiveTessellation::get_mesh_energy(const Eigen::VectorXd& v_flat)
{
    throw std::runtime_error("AT::get_mesh_energy should not be used");
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
        write_obj_displaced("smooth_" + std::to_string(it) + ".obj");

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


float AdaptiveTessellation::cumulated_per_face_error()
{
    std::vector<TriMesh::Tuple> tris_tuples = get_faces();
    float total_error = 0;
    lagrange::enable_fpe();
    std::vector<std::array<float, 6>> uv_triangles(tris_tuples.size());
    for (int i = 0; i < tris_tuples.size(); i++) {
        auto oriented_vids = oriented_tri_vids(tris_tuples[i]);
        for (int j = 0; j < 3; j++) {
            uv_triangles[tris_tuples[i].fid(*this)][2 * j + 0] =
                vertex_attrs[oriented_vids[j]].pos[0];
            uv_triangles[tris_tuples[i].fid(*this)][2 * j + 1] =
                vertex_attrs[oriented_vids[j]].pos[1];
        }
    }

    if (mesh_parameters.m_energy_type == ENERGY_TYPE::AREA_QUADRATURE) {
        std::vector<float> computed_errors(tris_tuples.size());
        m_texture_integral.get_error_per_triangle(uv_triangles, computed_errors);
        for (int i = 0; i < tris_tuples.size(); i++) {
            total_error += computed_errors[i];
        }
    } else if (mesh_parameters.m_energy_type == ENERGY_TYPE::AMIPS3D) {
        for (int i = 0; i < tris_tuples.size(); i++) {
            const Eigen::Vector2d v1 = Eigen::Vector2d(uv_triangles[i][0], uv_triangles[i][1]);
            const Eigen::Vector2d v2 = Eigen::Vector2d(uv_triangles[i][2], uv_triangles[i][3]);
            const Eigen::Vector2d v3 = Eigen::Vector2d(uv_triangles[i][4], uv_triangles[i][5]);
            total_error += get_amips3d_error_for_face(v1, v2, v3);
        }
    } else if (mesh_parameters.m_energy_type == ENERGY_TYPE::COMBINED) {
        std::vector<float> computed_errors(tris_tuples.size());
        m_texture_integral.get_error_per_triangle(uv_triangles, computed_errors);
        for (int i = 0; i < tris_tuples.size(); i++) {
            const Eigen::Vector2d v1 = Eigen::Vector2d(uv_triangles[i][0], uv_triangles[i][1]);
            const Eigen::Vector2d v2 = Eigen::Vector2d(uv_triangles[i][2], uv_triangles[i][3]);
            const Eigen::Vector2d v3 = Eigen::Vector2d(uv_triangles[i][4], uv_triangles[i][5]);
            auto amips_energy = get_amips3d_error_for_face(v1, v2, v3);
            total_error += (2 * computed_errors[i] +
                            pow(mesh_parameters.m_quality_threshold, 4) * amips_energy) /
                           (computed_errors[i] * amips_energy);
        }
    }
    return total_error;
}
