#include "extract_soup.hpp"

#include <igl/writeOFF.h>
#include <sec/ShortestEdgeCollapse.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Reader.hpp>

#include "ImageSimulationMesh.h"

namespace {

class VertexAttributes
{
public:
    wmtk::Vector3d m_posf;
};

class FaceAttributes
{
public:
    int64_t tag = -1;
};

class TetAttributes
{
public:
    int64_t tag = -1;
};

class TagMesh : public wmtk::TetMesh
{
public:
    TagMesh()
    {
        p_vertex_attrs = &m_vertex_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
    }

    ~TagMesh() {}
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    // using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    VertAttCol m_vertex_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;
    // EdgeAttCol m_edge_attribute;

    void output_mesh(std::string file);
    void output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond);

public:
    void output_init_tetmesh(std::string output_dir);
};

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

    data.resize(dim1, std::vector<std::vector<size_t>>(dim2, std::vector<size_t>(dim3)));

    for (int i = 0; i < dim1; ++i) {
        for (int j = 0; j < dim2; ++j) {
            for (int k = 0; k < dim3; ++k) {
                size_t value;
                file.read(reinterpret_cast<char*>(&value), sizeof(size_t));
                data[i][j][k] = value;
            }
        }
    }
}

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

    data.resize(dim1, std::vector<std::vector<size_t>>(dim2, std::vector<size_t>(dim3)));

    for (int i = 0; i < dim1; ++i) {
        for (int j = 0; j < dim2; ++j) {
            for (int k = 0; k < dim3; ++k) {
                size_t value;
                file >> value;
                data[i][j][k] = value;
            }
        }
    }
}


void readGmsh(
    const std::string& filename,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<Eigen::Vector4<size_t>>& tetrahedra)
{
    std::ifstream file(filename);
    std::string line;

    if (file.is_open()) {
        while (std::getline(file, line)) {
            if (line.find("Vertices") != std::string::npos) {
                break;
            }
        }

        size_t num_vertices;
        file >> num_vertices;

        vertices.reserve(num_vertices);

        for (size_t i = 0; i < num_vertices; ++i) {
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

        for (size_t i = 0; i < num_tetrahedons; ++i) {
            size_t v0, v1, v2, v3, dummy;
            file >> v0;
            file >> v1;
            file >> v2;
            file >> v3;
            file >> dummy;
            tetrahedra.push_back(Eigen::Vector4<size_t>(v0 - 1, v1 - 1, v2 - 1, v3 - 1));
        }

        file.close();
    } else {
        std::runtime_error("can't open the file!");
    }
}

/**
 * @brief Add tags to tet mesh using the barycenter of a tet and finding the cell it belongs to.
 */
void gmsh2hdf_tag(
    std::string volumetric_file,
    std::string gmsh_file,
    std::string output_file,
    double delta_x)
{
    using namespace wmtk;
    using Tuple = TetMesh::Tuple;
    using namespace components::image_simulation;

    std::vector<std::vector<std::vector<size_t>>> volumetric_data;
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector4<size_t>> tetrahedras;

    read_array_data(volumetric_data, volumetric_file);
    readGmsh(gmsh_file, vertices, tetrahedras);

    MatrixXi T;
    T.resize(tetrahedras.size(), 4);
    for (size_t i = 0; i < tetrahedras.size(); i++) {
        // T.row(i) = tetrahedras[i];
        T(i, 0) = tetrahedras[i].x();
        T(i, 1) = tetrahedras[i].y();
        T(i, 2) = tetrahedras[i].z();
        T(i, 3) = tetrahedras[i].w();
    }
    Eigen::MatrixXd V(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); i++) {
        // V.row(i) = vertices[i];
        V(i, 0) = vertices[i].x();
        V(i, 1) = vertices[i].y();
        V(i, 2) = vertices[i].z();
    }

    spdlog::info("V:{}\n", V.rows());
    spdlog::info("T:{}\n", T.rows());

    TagMesh mesh;
    mesh.init(T);

    for (const Tuple& t : mesh.get_vertices()) {
        const size_t vid = t.vid(mesh);
        mesh.m_vertex_attribute[vid].m_posf = V.row(vid);
    }

    // mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, mesh);
    //
    // auto tag_handle = mesh.register_attribute<int64_t>("tag", PrimitiveType::Tetrahedron, 1);
    // auto acc_tag = mesh.create_accessor<int64_t>(tag_handle);
    // auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    // auto acc_pos = mesh.create_accessor<double>(pos_handle);
    // auto bc_tag_handle = mesh.register_attribute<int64_t>("bc_tag", PrimitiveType::Tetrahedron,
    // 1); auto acc_bc_tag = mesh.create_accessor<int64_t>(bc_tag_handle);

    spdlog::info("Operating Tag...\n");
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
        if (idx_0 >= 0 && idx_0 < volumetric_data.size() && idx_1 > 0 &&
            idx_1 < volumetric_data[0].size() && idx_2 > 0 &&
            idx_2 < volumetric_data[0][0].size()) {
            // for tag
            int64_t intValue = volumetric_data[idx_0][idx_1][idx_2];
            mesh.m_tet_attribute[t.vid(mesh)].tag = intValue;
            // acc_tag.scalar_attribute(t) = intValue;
        }
    }

    spdlog::info("Operating position...\n");
    for (const Tuple& t : mesh.get_vertices()) {
        mesh.m_vertex_attribute[t.vid(mesh)].m_posf *= delta_x;
    }

    // spdlog::info("max_amips: {}\n", max_amips);
    // spdlog::info("before\n");
    // mesh.consolidate();
    // spdlog::info("after\n");

    // int cnt = 0;
    //
    // auto face_handle = mesh.register_attribute<int64_t>("surface", PrimitiveType::Triangle, 1);
    // auto acc_face = mesh.create_accessor<int64_t>(face_handle);
    //
    // for (const Tuple& face : mesh.get_all(wmtk::PrimitiveType::Triangle)) {
    //     if (!mesh.is_boundary_face(face)) {
    //         if (acc_tag.scalar_attribute(face) !=
    //             acc_tag.scalar_attribute(mesh.switch_tetrahedron(face))) {
    //             acc_face.scalar_attribute(face) = 1;
    //             cnt++;
    //         }
    //     }
    // }
    //
    // spdlog::info("cnt:{}\n", cnt);


    ///////////////////////////
    // write to MSG and VTU
    {
        // HDF5Writer writer(output_file + ".hdf5");
        // mesh.serialize(writer);
    }

    {
        // ParaviewWriter writer(output_file + ".vtu", "vertices", mesh, false, false, false, true);
        // mesh.serialize(writer);
    }
}

} // namespace


namespace wmtk::components::image_simulation {

void extract_triangle_soup_from_image(std::string filename, Eigen::MatrixXi& F, Eigen::MatrixXd& V)
{
    // std::vector<std::vector<std::vector<size_t>>> data;
    // read_array_data(data, filename);
    std::vector<std::vector<std::vector<size_t>>> data;
    read_array_data_ascii(data, filename);

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

} // namespace wmtk::components::image_simulation