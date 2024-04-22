#include "Extract.hpp"

#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <wmtk/utils/getRSS.h>
#include <filesystem>
#include <wmtk/TetMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "wmtk/function/simplex/AMIPS.hpp"
#include "wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp"
#include "wmtk/utils/Logger.hpp"

namespace wmtk::components::internal {
void extract_triangle_soup_from_image(
    std::string output_path,
    std::string filename,
    double delta_x,
    unsigned int max_level)
{
    std::vector<std::vector<std::vector<unsigned int>>> data;
    read_array_data(data, filename);
    unsigned int tri_num = 0;
    for (unsigned int i = 0; i < data.size() - 1; i++) {
        for (unsigned int j = 0; j < data[0].size() - 1; j++) {
            for (unsigned int k = 0; k < data[0][0].size() - 1; k++) {
                unsigned int cur_data = data[i][j][k];
                unsigned int right_data = data[i][j][k + 1];
                unsigned int ahead_data = data[i][j + 1][k];
                unsigned int top_data = data[i + 1][j][k];
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

    int target_value;
    Eigen::MatrixXi F(tri_num, 3);
    Eigen::MatrixXd V(3 * tri_num, 3);
    // int64_t itr = 0;
    // for (int64_t i = 0; i < data.size(); i++) {
    //     for (int64_t j = 0; j < data[0].size(); j++) {
    //         for (int64_t k = 0; k < data[0][0].size(); k++) {
    //             V.row(itr++) << i * delta_x, j * delta_x, k * delta_x;
    //         }
    //     }
    // }

    unsigned int cur_v_id = 0;
    unsigned int cur_f_id = 0;
    for (unsigned int i = 0; i < data.size() - 1; i++) {
        for (unsigned int j = 0; j < data[0].size() - 1; j++) {
            for (unsigned int k = 0; k < data[0][0].size() - 1; k++) {
                unsigned int cur_data = data[i][j][k];
                unsigned int right_data = data[i][j][k + 1];
                unsigned int ahead_data = data[i][j + 1][k];
                unsigned int top_data = data[i + 1][j][k];
                Eigen::RowVector3d v0(i, (j + 1), k);
                Eigen::RowVector3d v1(i, (j + 1), (k + 1));
                Eigen::RowVector3d v2(i, j, (k + 1));
                Eigen::RowVector3d v3((i + 1), (j + 1), k);
                Eigen::RowVector3d v4((i + 1), (j + 1), (k + 1));
                Eigen::RowVector3d v5((i + 1), j, (k + 1));
                Eigen::RowVector3d v6((i + 1), j, k);

                if (cur_data > right_data) {
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
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
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
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
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
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
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
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
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
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
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
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

    octree_add_points(V, max_level);

    V = V * delta_x;
    igl::writeOFF(output_path, V, F);
    // igl::writeOBJ(output_path, V, F);

    spdlog::info("max:{} B\n", wmtk::getPeakRSS());

    spdlog::info("V:{}\n", V.rows());
    spdlog::info("F:{}\n", F.rows());
}

void read_array_data(
    std::vector<std::vector<std::vector<unsigned int>>>& data,
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

    data.resize(
        dim1,
        std::vector<std::vector<unsigned int>>(dim2, std::vector<unsigned int>(dim3)));

    for (int i = 0; i < dim1; ++i) {
        for (int j = 0; j < dim2; ++j) {
            for (int k = 0; k < dim3; ++k) {
                unsigned int value;
                file.read(reinterpret_cast<char*>(&value), sizeof(unsigned int));
                data[i][j][k] = value;
            }
        }
    }
}

void octree_add_points(Eigen::MatrixXd& V, unsigned int max_level)
{
    Eigen::VectorXd min_vals = V.colwise().minCoeff();
    Eigen::VectorXd max_vals = V.colwise().maxCoeff();
    std::map<Eigen::Vector3d, bool, VectorComparer> record;
    std::vector<Eigen::Vector3d> sub_points;

    for (int64_t i = 0; i < V.rows(); i++) {
        sub_points.push_back(V.row(i));
    }

    octree_add_points(
        record,
        sub_points,
        max_level,
        min_vals.x(),
        min_vals.y(),
        min_vals.z(),
        max_vals.x(),
        max_vals.y(),
        max_vals.z());

    Eigen::MatrixXd append_V(record.size(), 3);
    record.size();
    unsigned int itr = 0;
    for (auto it = record.begin(); it != record.end(); ++it) {
        const Eigen::Vector3d& key = it->first;
        append_V.row(itr) = key;
        itr++;
    }
    Eigen::MatrixXd tempV = V;

    V.resize(tempV.rows() + append_V.rows(), 3);
    V << tempV, append_V;

    // V = append_V;

    spdlog::info("add points:{}\n", append_V.rows());
}

void octree_add_points(
    std::map<Eigen::Vector3d, bool, VectorComparer>& record,
    const std::vector<Eigen::Vector3d>& sub_points,
    unsigned int itr,
    double min_x,
    double min_y,
    double min_z,
    double max_x,
    double max_y,
    double max_z)
{
    if (itr == 0) {
        return;
    }
    if (sub_points.empty()) {
        return;
    }

    double mid_x = 0.5 * (min_x + max_x);
    double mid_y = 0.5 * (min_y + max_y);
    double mid_z = 0.5 * (min_z + max_z);

    // add 7 points
    Eigen::Vector3d p0(std::floor(mid_x) + 0.5, std::floor(mid_y) + 0.5, std::floor(mid_z) + 0.5);
    record[p0] = true;
    // Eigen::Vector3d p1(mid_x, mid_y, min_z);
    // Eigen::Vector3d p2(min_x, mid_y, mid_z);
    // Eigen::Vector3d p3(max_x, mid_y, mid_z);
    // Eigen::Vector3d p4(mid_x, mid_y, max_z);
    // Eigen::Vector3d p5(mid_x, max_y, mid_z);
    // Eigen::Vector3d p6(mid_x, min_y, mid_z);
    // if (!is_int(mid_x) && !is_int(mid_y) && !is_int(mid_z)) {
    //     record[p0] = true;
    // }
    // if (!is_int(mid_x) && !is_int(mid_y) && !is_int(min_z)) {
    //     record[p1] = true;
    // }
    // if (!is_int(min_x) && !is_int(mid_y) && !is_int(mid_z)) {
    //     record[p2] = true;
    // }
    // if (!is_int(max_x) && !is_int(mid_y) && !is_int(mid_z)) {
    //     record[p3] = true;
    // }
    // if (!is_int(mid_x) && !is_int(mid_y) && !is_int(max_z)) {
    //     record[p4] = true;
    // }
    // if (!is_int(mid_x) && !is_int(max_y) && !is_int(mid_z)) {
    //     record[p5] = true;
    // }
    // if (!is_int(mid_x) && !is_int(min_y) && !is_int(mid_z)) {
    //     record[p6] = true;
    // }


    // divide more sub-domains
    std::vector<Eigen::Vector3d> sub_points0;
    std::vector<Eigen::Vector3d> sub_points1;
    std::vector<Eigen::Vector3d> sub_points2;
    std::vector<Eigen::Vector3d> sub_points3;
    std::vector<Eigen::Vector3d> sub_points4;
    std::vector<Eigen::Vector3d> sub_points5;
    std::vector<Eigen::Vector3d> sub_points6;
    std::vector<Eigen::Vector3d> sub_points7;
    for (const Eigen::Vector3d& p : sub_points) {
        if (p.x() < mid_x && p.y() < mid_y && p.z() < mid_z) {
            sub_points0.push_back(p);
        } else if (p.x() > mid_x && p.y() < mid_y && p.z() < mid_z) {
            sub_points1.push_back(p);
        } else if (p.x() < mid_x && p.y() < mid_y && p.z() > mid_z) {
            sub_points2.push_back(p);
        } else if (p.x() > mid_x && p.y() < mid_y && p.z() > mid_z) {
            sub_points3.push_back(p);
        } else if (p.x() < mid_x && p.y() > mid_y && p.z() < mid_z) {
            sub_points4.push_back(p);
        } else if (p.x() > mid_x && p.y() > mid_y && p.z() < mid_z) {
            sub_points5.push_back(p);
        } else if (p.x() < mid_x && p.y() > mid_y && p.z() > mid_z) {
            sub_points6.push_back(p);
        } else if (p.x() > mid_x && p.y() > mid_y && p.z() > mid_z) {
            sub_points7.push_back(p);
        }
    }

    // recursively adding points
    octree_add_points(record, sub_points0, itr - 1, min_x, min_y, min_z, mid_x, mid_y, mid_z);


    octree_add_points(record, sub_points1, itr - 1, mid_x, min_y, min_z, max_x, mid_y, mid_z);


    octree_add_points(record, sub_points2, itr - 1, min_x, min_y, mid_z, mid_x, mid_y, max_z);


    octree_add_points(record, sub_points3, itr - 1, mid_x, min_y, mid_z, max_x, mid_y, max_z);


    octree_add_points(record, sub_points4, itr - 1, min_x, mid_y, min_z, mid_x, max_y, mid_z);


    octree_add_points(record, sub_points5, itr - 1, mid_x, mid_y, min_z, max_x, max_y, mid_z);


    octree_add_points(record, sub_points6, itr - 1, min_x, mid_y, mid_z, mid_x, max_y, max_z);


    octree_add_points(record, sub_points7, itr - 1, mid_x, mid_y, mid_z, max_x, max_y, max_z);
}


bool is_int(double v)
{
    return floor(v) == ceil(v);
}

void readGmsh(
    const std::string& filename,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<Eigen::Vector4<unsigned int>>& tetrahedra)
{
    std::ifstream file(filename);
    std::string line;

    if (file.is_open()) {
        while (std::getline(file, line)) {
            if (line.find("Vertices") != std::string::npos) {
                break;
            }
        }

        unsigned int num_vertices;
        file >> num_vertices;

        vertices.reserve(num_vertices);

        for (unsigned int i = 0; i < num_vertices; ++i) {
            double p1, p2, p3, dummy;
            file >> p1 >> p2 >> p3 >> dummy;
            vertices.push_back(Eigen::Vector3d(p1, p2, p3));
        }

        while (std::getline(file, line)) {
            if (line.find("Tetrahedra") != std::string::npos) {
                break;
            }
        }
        int num_tetrahedons;
        file >> num_tetrahedons;

        tetrahedra.reserve(num_tetrahedons);

        for (unsigned int i = 0; i < num_tetrahedons; ++i) {
            unsigned int v0, v1, v2, v3, dummy;
            file >> v0;
            file >> v1;
            file >> v2;
            file >> v3;
            file >> dummy;
            tetrahedra.push_back(Eigen::Vector4<unsigned int>(v0 - 1, v1 - 1, v2 - 1, v3 - 1));
        }

        file.close();
    } else {
        std::runtime_error("can't open the file!");
    }
}

void gmsh2hdf_tag(std::string volumetric_file, std::string gmsh_file, std::string output_file)
{
    std::vector<std::vector<std::vector<unsigned int>>> volumetric_data;
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector4<unsigned int>> tetrahedras;

    read_array_data(volumetric_data, volumetric_file);
    readGmsh(gmsh_file, vertices, tetrahedras);

    RowVectors4l T;
    T.resize(tetrahedras.size(), 4);
    for (unsigned int i = 0; i < tetrahedras.size(); i++) {
        // T.row(i) = tetrahedras[i];
        T(i, 0) = tetrahedras[i].x();
        T(i, 1) = tetrahedras[i].y();
        T(i, 2) = tetrahedras[i].z();
        T(i, 3) = tetrahedras[i].w();
    }
    Eigen::MatrixXd V(vertices.size(), 3);
    for (unsigned int i = 0; i < vertices.size(); i++) {
        // V.row(i) = vertices[i];
        V(i, 0) = vertices[i].x();
        V(i, 1) = vertices[i].y();
        V(i, 2) = vertices[i].z();
    }

    spdlog::info("V:{}\n", V.rows());
    spdlog::info("T:{}\n", T.rows());

    TetMesh mesh;
    mesh.initialize(T);

    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, mesh);

    auto tag_handle = mesh.register_attribute<int64_t>("tag", PrimitiveType::Tetrahedron, 1);
    auto acc_tag = mesh.create_accessor<int64_t>(tag_handle);
    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto acc_pos = mesh.create_accessor<double>(pos_handle);

    for (const Tuple& t : mesh.get_all(PrimitiveType::Tetrahedron)) {
        auto v0 = acc_pos.vector_attribute(t);
        auto v1 = acc_pos.vector_attribute(mesh.switch_vertex(t));
        auto v2 = acc_pos.vector_attribute(mesh.switch_vertex(mesh.switch_edge(t)));
        auto v3 =
            acc_pos.vector_attribute(mesh.switch_vertex(mesh.switch_edge(mesh.switch_face(t))));
        auto center = (v0 + v1 + v2 + v3) * 0.25;
        int idx_0 = std::floor(center.x());
        int idx_1 = std::floor(center.y());
        int idx_2 = std::floor(center.z());
        if (idx_0 >= 0 && idx_0 < volumetric_data.size() && idx_1 > 0 &&
            idx_1 < volumetric_data[0].size() && idx_2 > 0 &&
            idx_2 < volumetric_data[0][0].size()) {
            int64_t v = volumetric_data[idx_0][idx_1][idx_2];
            acc_tag.scalar_attribute(t) = v;
        }
    }

    // auto amips_attribute =
    //     mesh.register_attribute<double>("wildmeshing_amips", mesh.top_simplex_type(), 1);
    // auto amips_accessor = mesh.create_accessor(amips_attribute.as<double>());
    // auto compute_amips = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
    //     assert(P.rows() == 2 || P.rows() == 3); // rows --> attribute dimension
    //     assert(P.cols() == P.rows() + 1);
    //     if (P.cols() == 3) {
    //         // triangle
    //         assert(P.rows() == 2);
    //         std::array<double, 6> pts;
    //         for (size_t i = 0; i < 3; ++i) {
    //             for (size_t j = 0; j < 2; ++j) {
    //                 pts[2 * i + j] = P(j, i);
    //             }
    //         }
    //         const double a = wmtk::function::Tri_AMIPS_energy(pts);
    //         return Eigen::VectorXd::Constant(1, a);
    //     } else {
    //         // tet
    //         assert(P.rows() == 3);
    //         std::array<double, 12> pts;
    //         for (size_t i = 0; i < 4; ++i) {
    //             for (size_t j = 0; j < 3; ++j) {
    //                 pts[3 * i + j] = P(j, i);
    //             }
    //         }
    //         const double a = wmtk::function::Tet_AMIPS_energy(pts);
    //         return Eigen::VectorXd::Constant(1, a);
    //     }
    // };
    // auto amips_update =
    //     std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
    //         amips_attribute,
    //         pos_handle,
    //         compute_amips);
    // amips_update->run_on_all();

    // {
    //     ParaviewWriter writer(output_file, "vertices", mesh, false, false, false, true);
    //     mesh.serialize(writer);
    // }

    // double max_amips = 0;
    // for (const Tuple t : mesh.get_all(wmtk::PrimitiveType::Tetrahedron)) {
    //     max_amips = std::max(max_amips, amips_accessor.scalar_attribute(t));
    // }

    // spdlog::info("max_amips: {}\n", max_amips);
    mesh.consolidate();
    auto face_handle =
        mesh.register_attribute<int64_t>("face_constraint_tag", PrimitiveType::Triangle, 1);
    auto acc_face = mesh.create_accessor<int64_t>(face_handle);

    for (const Tuple& face : mesh.get_all(wmtk::PrimitiveType::Triangle)) {
        if (!mesh.is_boundary_face(face)) {
            if (acc_tag.scalar_attribute(face) !=
                acc_tag.scalar_attribute(mesh.switch_tetrahedron(face))) {
                acc_face.scalar_attribute(face) = 1;
            }
        }
    }

    {
        auto child_mesh = multimesh::utils::extract_and_register_child_mesh_from_tag(
            mesh,
            "face_constraint_tag",
            1,
            wmtk::PrimitiveType::Triangle);
        HDF5Writer writer(output_file + ".constraint.hdf5");
        child_mesh->serialize(writer);
    }

    {
        HDF5Writer writer(output_file + ".hdf5");
        mesh.serialize(writer);
    }
    spdlog::info("max:{} B\n", wmtk::getPeakRSS());
}
} // namespace wmtk::components::internal
