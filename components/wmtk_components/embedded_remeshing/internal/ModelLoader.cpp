#include "ModelLoader.hpp"

namespace wmtk::components::internal {

void tri_divide_a(RowVectors3l& tris, long grid_x, long grid_y, long id0, long Toffset)
{
    // 0       1
    // *-------*
    // | \___ 1|
    // | 0   \_|
    // *-------*
    // 2       3
    long id1 = id0 + 1;
    long id2 = id0 + grid_x + 1;
    long id3 = id2 + 1;
    tris.row(Toffset) << id0, id2, id3;
    tris.row(Toffset + 1) << id0, id3, id1;
    // // this is a circle
    // if ((i - 10) * (i - 10) + (j - 10) * (j - 10) < 25) {
    //     labels((grid_x * i + j) * 2, 0) = input_tag_value;
    //     labels((grid_x * i + j) * 2 + 1, 0) = input_tag_value;
    // } else {
    //     labels((grid_x * i + j) * 2, 0) = embedding_tag_value;
    //     labels((grid_x * i + j) * 2 + 1, 0) = embedding_tag_value;
    // }
}

void tri_divide_b(RowVectors3l& tris, long grid_x, long grid_y, long id0, long Toffset)
{
    // 0       1
    // *-------*
    // | 0___/ |
    // | /   1 |
    // *-------*
    // 2       3
    long id1 = id0 + 1;
    long id2 = id0 + grid_x + 1;
    long id3 = id2 + 1;
    tris.row(Toffset) << id0, id2, id1;
    tris.row(Toffset + 1) << id2, id3, id1;
}

void load_matrix_in_trimesh(TriMesh& mesh, const std::vector<std::vector<long>>& labels)
{
    const long embedding_tag_value = 0;
    const long input_tag_value = 1;

    spdlog::info("Begin to load the model!");
    long grid_x, grid_y;
    grid_x = labels[0].size();
    grid_y = labels.size();
    Eigen::MatrixXd V((grid_x + 1) * (grid_y + 1), 3);
    for (long i = 0; i < grid_y + 1; ++i) {
        for (long j = 0; j < grid_x + 1; ++j) {
            V.row(i * (grid_x + 1) + j) << j, i, 0;
        }
    }

    RowVectors3l tris;
    tris.resize(2 * grid_x * grid_y, 3);
    std::vector<long> face_label_record_list;
    face_label_record_list.reserve(2 * grid_x * grid_y);
    bool y_range_type = true;
    bool x_range_type = true;
    for (long i = 0; i < grid_y; ++i) {
        x_range_type = y_range_type;
        for (long j = 0; j < grid_x; ++j) {
            long id0 = i * (grid_x + 1) + j;
            long Toffset = (grid_x * i + j) * 2;
            if (x_range_type) {
                tri_divide_a(tris, grid_x, grid_y, id0, Toffset);
            } else {
                tri_divide_b(tris, grid_x, grid_y, id0, Toffset);
            }
            if (labels[i][j] == 1) {
                // input
                face_label_record_list.push_back(input_tag_value);
                face_label_record_list.push_back(input_tag_value);
            } else {
                // embedding
                face_label_record_list.push_back(embedding_tag_value);
                face_label_record_list.push_back(embedding_tag_value);
            }
            x_range_type = !x_range_type;
        }
        y_range_type = !y_range_type;
    }

    mesh.initialize(tris);
    wmtk::mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);
    MeshAttributeHandle<long> vertex_tag_handle = mesh.register_attribute<long>(
        "vertex_tag",
        PrimitiveType::Vertex,
        1,
        false,
        embedding_tag_value);
    MeshAttributeHandle<long> edge_tag_handle = mesh.register_attribute<long>(
        "edge_tag",
        PrimitiveType::Edge,
        1,
        false,
        embedding_tag_value);
    MeshAttributeHandle<long> face_tag_handle = mesh.register_attribute<long>(
        "face_tag",
        PrimitiveType::Face,
        1,
        false,
        embedding_tag_value);

    Accessor<long> acc_vertex_tag = mesh.create_accessor(vertex_tag_handle);
    Accessor<long> acc_edge_tag = mesh.create_accessor(edge_tag_handle);
    Accessor<long> acc_face_tag = mesh.create_accessor(face_tag_handle);

    // load input face label
    {
        size_t i = 0;
        for (const Tuple& f : mesh.get_all(PrimitiveType::Face)) {
            acc_face_tag.scalar_attribute(f) = face_label_record_list[i];
            ++i;
        }
    }

    // load input edge and vertex label
    {
        for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
            if (mesh.is_boundary(e)) {
                continue;
            } else if (
                acc_face_tag.scalar_attribute(e) !=
                acc_face_tag.scalar_attribute(mesh.switch_face(e))) {
                acc_edge_tag.scalar_attribute(e) = input_tag_value;
                acc_vertex_tag.scalar_attribute(e) = input_tag_value;
                acc_vertex_tag.scalar_attribute(mesh.switch_vertex(e)) = input_tag_value;
            }
        }
    }
}

void tet_divide_a(RowVectors4l& tets, long grid_x, long grid_y, long id0, long Toffset)
{
    long id1, id2, id3, id4, id5, id6, id7;
    id1 = id0 + 1;
    id2 = id0 + grid_x + 1;
    id3 = id2 + 1;
    id4 = id0 + (grid_x + 1) * (grid_y + 1);
    id5 = id4 + 1;
    id6 = id4 + grid_x + 1;
    id7 = id6 + 1;
    tets.row(Toffset) << id0, id1, id2, id4;
    tets.row(Toffset + 1) << id4, id1, id7, id5;
    tets.row(Toffset + 2) << id2, id1, id7, id3;
    tets.row(Toffset + 3) << id4, id2, id7, id6;
    tets.row(Toffset + 4) << id4, id1, id2, id7;
}

void tet_divide_b(RowVectors4l& tets, long grid_x, long grid_y, long id0, long Toffset)
{
    long id1, id2, id3, id4, id5, id6, id7;
    id1 = id0 + 1;
    id2 = id0 + grid_x + 1;
    id3 = id2 + 1;
    id4 = id0 + (grid_x + 1) * (grid_y + 1);
    id5 = id4 + 1;
    id6 = id4 + grid_x + 1;
    id7 = id6 + 1;
    tets.row(Toffset) << id0, id1, id3, id5;
    tets.row(Toffset + 1) << id4, id0, id5, id6;
    tets.row(Toffset + 2) << id6, id5, id7, id3;
    tets.row(Toffset + 3) << id6, id0, id3, id2;
    tets.row(Toffset + 4) << id6, id0, id5, id3;
}

void load_matrix_in_tetmesh(
    TetMesh& mesh,
    const std::vector<std::vector<std::vector<long>>>& labels)
{
    const long embedding_tag_value = 0;
    const long input_tag_value = 1;

    spdlog::info("Begin to load the model!");
    long grid_x, grid_y, grid_z;
    grid_x = labels[0][0].size();
    grid_y = labels[0].size();
    grid_z = labels.size();
    Eigen::MatrixXd V((grid_x + 1) * (grid_y + 1) * (grid_z + 1), 3);
    for (long i = 0; i < grid_z + 1; ++i) {
        for (long j = 0; j < grid_y + 1; ++j) {
            for (long k = 0; k < grid_x + 1; ++k) {
                V.row(i * (grid_x + 1) * (grid_y + 1) + j * (grid_x + 1) + k) << k, j, i;
            }
        }
    }

    RowVectors4l tets;
    tets.resize(5 * grid_x * grid_y * grid_z, 4);

    std::vector<long> tet_label_record_list;
    tet_label_record_list.reserve(5 * grid_x * grid_y * grid_z);
    bool z_range_type = true;
    bool y_range_type = true;
    bool x_range_type = true;
    for (long i = 0; i < grid_z; ++i) {
        y_range_type = z_range_type;
        for (long j = 0; j < grid_y; ++j) {
            x_range_type = y_range_type;
            for (long k = 0; k < grid_x; ++k) {
                long id0 = i * ((grid_x + 1) * (grid_y + 1)) + j * (grid_x + 1) + k;
                long Toffset = (grid_x * grid_y * i + grid_x * j + k) * 5;
                if (x_range_type) {
                    tet_divide_a(tets, grid_x, grid_y, id0, Toffset);
                } else {
                    tet_divide_b(tets, grid_x, grid_y, id0, Toffset);
                }
                if (labels[i][j][k] == 1) {
                    // input
                    tet_label_record_list.push_back(input_tag_value);
                    tet_label_record_list.push_back(input_tag_value);
                    tet_label_record_list.push_back(input_tag_value);
                    tet_label_record_list.push_back(input_tag_value);
                    tet_label_record_list.push_back(input_tag_value);
                } else {
                    // embedding
                    tet_label_record_list.push_back(embedding_tag_value);
                    tet_label_record_list.push_back(embedding_tag_value);
                    tet_label_record_list.push_back(embedding_tag_value);
                    tet_label_record_list.push_back(embedding_tag_value);
                    tet_label_record_list.push_back(embedding_tag_value);
                }
                x_range_type = !x_range_type;
            }
            y_range_type = !y_range_type;
        }
        z_range_type = !z_range_type;
    }

    mesh.initialize(tets);
    wmtk::mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);
    MeshAttributeHandle<long> vertex_tag_handle = mesh.register_attribute<long>(
        "vertex_tag",
        PrimitiveType::Vertex,
        1,
        false,
        embedding_tag_value);
    MeshAttributeHandle<long> edge_tag_handle = mesh.register_attribute<long>(
        "edge_tag",
        PrimitiveType::Edge,
        1,
        false,
        embedding_tag_value);
    MeshAttributeHandle<long> face_tag_handle = mesh.register_attribute<long>(
        "face_tag",
        PrimitiveType::Face,
        1,
        false,
        embedding_tag_value);
    MeshAttributeHandle<long> tetrahedron_tag_handle = mesh.register_attribute<long>(
        "tetrahedron_tag",
        PrimitiveType::Tetrahedron,
        1,
        false,
        embedding_tag_value);

    Accessor<long> acc_vertex_tag = mesh.create_accessor(vertex_tag_handle);
    Accessor<long> acc_edge_tag = mesh.create_accessor(edge_tag_handle);
    Accessor<long> acc_face_tag = mesh.create_accessor(face_tag_handle);
    Accessor<long> acc_tetrahedron_tag = mesh.create_accessor(tetrahedron_tag_handle);

    // load input tetrahedon label
    {
        size_t i = 0;
        for (const Tuple& t : mesh.get_all(PrimitiveType::Tetrahedron)) {
            acc_tetrahedron_tag.scalar_attribute(t) = tet_label_record_list[i];
            ++i;
        }
    }

    // load input face edge and vertex label
    {
        for (const Tuple& f : mesh.get_all(PrimitiveType::Face)) {
            if (mesh.is_boundary_face(f)) {
                continue;
            } else if (
                acc_tetrahedron_tag.scalar_attribute(f) !=
                acc_tetrahedron_tag.scalar_attribute(mesh.switch_tetrahedron(f))) {
                acc_face_tag.scalar_attribute(f) = input_tag_value;
                acc_edge_tag.scalar_attribute(f) = input_tag_value;
                acc_edge_tag.scalar_attribute(mesh.switch_edge(f)) = input_tag_value;
                acc_edge_tag.scalar_attribute(mesh.switch_edge(mesh.switch_vertex(f))) =
                    input_tag_value;
                acc_vertex_tag.scalar_attribute(f) = input_tag_value;
                acc_vertex_tag.scalar_attribute(mesh.switch_vertex(f)) = input_tag_value;
                acc_vertex_tag.scalar_attribute(mesh.switch_vertex(mesh.switch_edge(f))) =
                    input_tag_value;
            }
        }
    }
}
} // namespace wmtk::components::internal