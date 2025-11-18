#include "extract_soup.hpp"

#include <igl/writeOFF.h>
#include <sec/ShortestEdgeCollapse.h>
#include <wmtk/utils/InsertTriangleUtils.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Reader.hpp>

#include "ImageSimulationMesh.h"

namespace {

void read_array_data(
    std::vector<std::vector<std::vector<size_t>>>& data,
    const std::string& filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for reading." << std::endl;
        return;
    }

    int dim1, dim2, dim3;
    file.read(reinterpret_cast<char*>(&dim1), sizeof(int));
    file.read(reinterpret_cast<char*>(&dim2), sizeof(int));
    file.read(reinterpret_cast<char*>(&dim3), sizeof(int));

    data.resize(dim3, std::vector<std::vector<size_t>>(dim2, std::vector<size_t>(dim1)));

    for (int k = 0; k < dim1; ++k) {
        for (int j = 0; j < dim2; ++j) {
            for (int i = 0; i < dim3; ++i) {
                size_t value;
                file.read(reinterpret_cast<char*>(&value), sizeof(size_t));
                data[i][j][k] = value;
            }
        }
    }
}

} // namespace


namespace wmtk::components::image_simulation {

void read_array_data_ascii(
    std::vector<std::vector<std::vector<size_t>>>& data,
    const std::string& filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        wmtk::log_and_throw_error("Error: Unable to open file {}", filename);
    }

    int dim1, dim2, dim3;
    file >> dim1;
    file >> dim2;
    file >> dim3;

    data.resize(dim3, std::vector<std::vector<size_t>>(dim2, std::vector<size_t>(dim1)));

    for (int k = 0; k < dim1; ++k) {
        for (int j = 0; j < dim2; ++j) {
            for (int i = 0; i < dim3; ++i) {
                size_t value;
                file >> value;
                data[i][j][k] = value;
            }
        }
    }
}

void extract_triangle_soup_from_image(std::string filename, Eigen::MatrixXi& F, Eigen::MatrixXd& V)
{
    // std::vector<std::vector<std::vector<size_t>>> data;
    // read_array_data(data, filename);
    std::vector<std::vector<std::vector<size_t>>> data;
    read_array_data_ascii(data, filename);

    extract_triangle_soup_from_image(data, F, V);
}

void extract_triangle_soup_from_image(
    const std::vector<std::vector<std::vector<size_t>>>& data,
    Eigen::MatrixXi& F,
    Eigen::MatrixXd& V)
{
    size_t tri_num = 0;
    for (size_t i = 0; i < data.size() - 1; i++) {
        for (size_t j = 0; j < data[0].size() - 1; j++) {
            for (size_t k = 0; k < data[0][0].size() - 1; k++) {
                size_t cur_data = data[i][j][k];
                size_t right_data = data[i][j][k + 1];
                size_t ahead_data = data[i][j + 1][k];
                size_t top_data = data[i + 1][j][k];
                if (cur_data != right_data) {
                    tri_num += 2;
                }
                if (cur_data != ahead_data) {
                    tri_num += 2;
                }
                if (cur_data != top_data) {
                    tri_num += 2;
                }
            }
        }
    }

    F.resize(tri_num, 3);
    V.resize(3 * tri_num, 3);

    size_t cur_v_id = 0;
    size_t cur_f_id = 0;
    for (size_t i = 0; i < data.size() - 1; i++) {
        for (size_t j = 0; j < data[0].size() - 1; j++) {
            for (size_t k = 0; k < data[0][0].size() - 1; k++) {
                size_t cur_data = data[i][j][k];
                size_t right_data = data[i][j][k + 1];
                size_t ahead_data = data[i][j + 1][k];
                size_t top_data = data[i + 1][j][k];
                Eigen::RowVector3d v0(i, (j + 1), k);
                Eigen::RowVector3d v1(i, (j + 1), (k + 1));
                Eigen::RowVector3d v2(i, j, (k + 1));
                Eigen::RowVector3d v3((i + 1), (j + 1), k);
                Eigen::RowVector3d v4((i + 1), (j + 1), (k + 1));
                Eigen::RowVector3d v5((i + 1), j, (k + 1));
                Eigen::RowVector3d v6((i + 1), j, k);

                if (cur_data > right_data) {
                    size_t vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v2;
                    V.row(vid1) = v5;
                    V.row(vid2) = v4;
                    V.row(vid3) = v2;
                    V.row(vid4) = v4;
                    V.row(vid5) = v1;

                } else if (cur_data < right_data) {
                    size_t vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v1;
                    V.row(vid1) = v4;
                    V.row(vid2) = v2;
                    V.row(vid3) = v4;
                    V.row(vid4) = v5;
                    V.row(vid5) = v2;
                }

                if (cur_data > ahead_data) {
                    size_t vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v0;
                    V.row(vid1) = v1;
                    V.row(vid2) = v3;
                    V.row(vid3) = v1;
                    V.row(vid4) = v4;
                    V.row(vid5) = v3;
                } else if (cur_data < ahead_data) {
                    size_t vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v0;
                    V.row(vid1) = v3;
                    V.row(vid2) = v1;
                    V.row(vid3) = v3;
                    V.row(vid4) = v4;
                    V.row(vid5) = v1;
                }

                if (cur_data > top_data) {
                    size_t vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v6;
                    V.row(vid1) = v3;
                    V.row(vid2) = v5;
                    V.row(vid3) = v3;
                    V.row(vid4) = v4;
                    V.row(vid5) = v5;
                } else if (cur_data < top_data) {
                    size_t vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v3;
                    V.row(vid1) = v5;
                    V.row(vid2) = v4;
                    V.row(vid3) = v3;
                    V.row(vid4) = v6;
                    V.row(vid5) = v5;
                }
            }
        }
    }

    // V = V * delta_x;
    // igl::writeOFF(output_path, V, F);
    // igl::writeOBJ(output_path, V, F);

    wmtk::logger().info("V:{}", V.rows());
    wmtk::logger().info("F:{}", F.rows());
}

void tag_tets_from_image(const std::string& filename, ImageSimulationMesh& mesh)
{
    logger().info("Tag tets");
    using Tuple = TetMesh::Tuple;

    std::vector<std::vector<std::vector<size_t>>> volumetric_data;
    read_array_data_ascii(volumetric_data, filename);

    for (const Tuple& t : mesh.get_tets()) {
        const auto vids = mesh.oriented_tet_vids(t);
        const Vector3d v0 = mesh.m_vertex_attribute[vids[0]].m_posf;
        const Vector3d v1 = mesh.m_vertex_attribute[vids[1]].m_posf;
        const Vector3d v2 = mesh.m_vertex_attribute[vids[2]].m_posf;
        const Vector3d v3 = mesh.m_vertex_attribute[vids[3]].m_posf;

        const Vector3d center = (v0 + v1 + v2 + v3) * 0.25;
        const int idx_0 = std::floor(center.x());
        const int idx_1 = std::floor(center.y());
        const int idx_2 = std::floor(center.z());
        if (idx_0 >= 0 && idx_0 < volumetric_data.size() && idx_1 >= 0 &&
            idx_1 < volumetric_data[0].size() && idx_2 >= 0 &&
            idx_2 < volumetric_data[0][0].size()) {
            // for tag
            int64_t intValue = volumetric_data[idx_0][idx_1][idx_2];
            mesh.m_tet_attribute[t.tid(mesh)].tags.push_back(intValue);
        }
    }
}

void image_to_tagged_tets(const std::string& filename, MatrixXd& V, MatrixXi& T, VectorXi T_tags)
{
    //// extract surface from image
    //{
    //    // read raw image data + create triangle soup
    //    Eigen::MatrixXi F;
    //    Eigen::MatrixXd V;
    //    extract_triangle_soup_from_image(filename, F, V);
    //    igl::writeOFF("triangle_soup_fine.off", V, F);
    //}
    //
    // std::vector<Eigen::Vector3d> verts;
    // std::vector<std::array<size_t, 3>> tris;
    // std::vector<size_t> modified_nonmanifold_v;
    // std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    //
    //// read triangle soup
    //{
    //    const double remove_duplicate_eps = 0.01;
    //    wmtk::stl_to_manifold_wmtk_input(
    //        "triangle_soup_fine.off",
    //        remove_duplicate_eps,
    //        box_minmax,
    //        verts,
    //        tris,
    //        modified_nonmanifold_v);
    //}
    //
    // std::vector<Eigen::Vector3d> v_simplified;
    // std::vector<std::array<size_t, 3>> f_simplified;
    //
    //// simplification
    // app::sec::ShortestEdgeCollapse surf_mesh(verts, 0, false);
    //{
    //    // must be a small envelope to ensure correct tet tags later on
    //    surf_mesh.create_mesh(verts.size(), tris, modified_nonmanifold_v, 0.1);
    //    assert(surf_mesh.check_mesh_connectivity_validity());
    //
    //    wmtk::logger().info("input {} simplification", filename);
    //    surf_mesh.collapse_shortest(0);
    //    surf_mesh.consolidate_mesh();
    //    surf_mesh.write_triangle_mesh("triangle_soup_coarse.off");
    //
    //    //// get the simplified input
    //    v_simplified.resize(surf_mesh.vert_capacity());
    //    f_simplified.resize(surf_mesh.tri_capacity());
    //    for (const auto& t : surf_mesh.get_vertices()) {
    //        const size_t i = t.vid(surf_mesh);
    //        v_simplified[i] = surf_mesh.vertex_attrs[i].pos;
    //    }
    //
    //    for (const auto& t : surf_mesh.get_faces()) {
    //        const auto i = t.fid(surf_mesh);
    //        const auto vs = surf_mesh.oriented_tri_vids(t);
    //        for (int j = 0; j < 3; j++) {
    //            f_simplified[i][j] = vs[j];
    //        }
    //    }
    //}
    //
    // wmtk::remove_duplicates(v_simplified, f_simplified, 0.01);
    //
    ///////////////////////////////////////////////////////
    //
    // igl::Timer timer;
    // timer.start();
    //
    //// triangle insertion with volumeremesher on the simplified mesh
    // std::vector<Vector3r> v_rational;
    // std::vector<std::array<size_t, 3>> facets;
    // std::vector<bool> is_v_on_input;
    // std::vector<std::array<size_t, 4>> tets;
    // std::vector<bool> tet_face_on_input_surface;
    //
    // std::cout << "vsimp size: " << v_simplified.size() << std::endl;
    // std::cout << "fsimp size: " << f_simplified.size() << std::endl;
    //
    // igl::Timer insertion_timer;
    // insertion_timer.start();
    //
    //{
    //     ImageSimulationMesh mesh_for_insertion(params, *ptr_env, surf_mesh.m_envelope,
    //     NUM_THREADS); mesh_for_insertion.insertion_by_volumeremesher(
    //         v_simplified,
    //         f_simplified,
    //         v_rational,
    //         facets,
    //         is_v_on_input,
    //         tets,
    //         tet_face_on_input_surface);
    // }
    //
    //// generate new mesh
    // image_simulation::ImageSimulationMesh mesh(params, *ptr_env, surf_mesh.m_envelope,
    // NUM_THREADS);
    //
    // mesh.init_from_Volumeremesher(v_rational, is_v_on_input, tets, tet_face_on_input_surface);
    //
    // const double insertion_time = insertion_timer.getElapsedTime();
    // wmtk::logger().info("volume remesher insertion time: {}s", insertion_time);
    //
    // mesh.consolidate_mesh();
    //{
    //     int num_parts = mesh.flood_fill();
    //     logger().info("flood fill parts {}", num_parts);
    // }
    //
    //// add tags
    // tag_tets_from_image(input_paths[0], mesh);
    //
    // mesh.write_vtu(params.output_path + "_0.vtu");
}

} // namespace wmtk::components::image_simulation