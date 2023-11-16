#include "embedded_remeshing.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>


#include <wmtk/utils/Logger.hpp>

#include <wmtk/utils/mesh_utils.hpp>
#include "internal/EmbeddedRemeshing.hpp"
#include "internal/EmbeddedRemeshingOptions.hpp"

namespace wmtk {
namespace components {
void embedded_remeshing(std::map<std::string, std::filesystem::path>& files)
{
    const long embedding_tag_value = 0;
    const long input_tag_value = 1;
    const long split_tag_value = 2; // offset

    spdlog::info("Pipeline begin!");
    int x, y;
    x = 20;
    y = 20;
    Eigen::MatrixXi labels_matrix(x, y);
    Eigen::MatrixXi labels(x * y * 2, 1);
    int grid_y = labels_matrix.rows();
    int grid_x = labels_matrix.cols();
    Eigen::MatrixXd V((grid_x + 1) * (grid_y + 1), 3);
    for (int i = 0; i < grid_y + 1; ++i) {
        for (int j = 0; j < grid_x + 1; ++j) {
            V.row(i * (grid_x + 1) + j) << j, i, 0;
        }
    }
    RowVectors3l tris;
    tris.resize(2 * grid_x * grid_y, 3);
    for (int i = 0; i < grid_y; ++i) {
        for (int j = 0; j < grid_x; ++j) {
            int id0, id1, id2, id3;
            // 0       1
            // *-------*
            // | \___ 1|
            // | 0   \_|
            // *-------*
            // 2       3
            id0 = i * (grid_x + 1) + j;
            id1 = id0 + 1;
            id2 = id0 + grid_x + 1;
            id3 = id2 + 1;
            tris.row((grid_x * i + j) * 2) << id0, id2, id3;
            tris.row((grid_x * i + j) * 2 + 1) << id0, id3, id1;
            // this is a circle
            if ((i - 10) * (i - 10) + (j - 10) * (j - 10) < 25) {
                labels((grid_x * i + j) * 2, 0) = input_tag_value;
                labels((grid_x * i + j) * 2 + 1, 0) = input_tag_value;
            } else {
                labels((grid_x * i + j) * 2, 0) = embedding_tag_value;
                labels((grid_x * i + j) * 2 + 1, 0) = embedding_tag_value;
            }
        }
    }
    // TriMesh mesh;
    // // std::cout << tris;
    // mesh.initialize(tris);
    // spdlog::info("{}", mesh.get_all(PrimitiveType::Vertex).size());

    // mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);
    // MeshAttributeHandle<double> pos_handle =
    //     mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    // MeshAttributeHandle<long> vertex_tag_handle = mesh.register_attribute<long>(
    //     "vertex_tag",
    //     PrimitiveType::Vertex,
    //     1,
    //     false,
    //     embedding_tag_value);
    // MeshAttributeHandle<long> edge_tag_handle = mesh.register_attribute<long>(
    //     "edge_tag",
    //     PrimitiveType::Edge,
    //     1,
    //     false,
    //     embedding_tag_value);
    // MeshAttributeHandle<long> pixel_handle = mesh.register_attribute<long>(
    //     "pixel_value",
    //     PrimitiveType::Face,
    //     1,
    //     false,
    //     embedding_tag_value);
    // Accessor<long> acc_vertex_tag = mesh.create_accessor(vertex_tag_handle);
    // Accessor<long> acc_edge_tag = mesh.create_accessor(edge_tag_handle);
    // Accessor<long> acc_face_tag = mesh.create_accessor(pixel_handle);

    // {
    //     int i = 0;
    //     for (const Tuple& t : mesh.get_all(PrimitiveType::Face)) {
    //         acc_face_tag.scalar_attribute(t) = labels(i, 0);
    //         ++i;
    //     }
    // }

    // for (const Tuple& t : mesh.get_all(PrimitiveType::Edge)) {
    //     if (!mesh.is_boundary_edge(t)) {
    //         long t0, t1;
    //         t0 = acc_face_tag.scalar_attribute(t);
    //         t1 = acc_face_tag.scalar_attribute(mesh.switch_face(t));
    //         if (t0 != t1) {
    //             acc_edge_tag.scalar_attribute(t) = input_tag_value;
    //             acc_vertex_tag.scalar_attribute(t) = input_tag_value;
    //             acc_vertex_tag.scalar_attribute(mesh.switch_vertex(t)) = input_tag_value;
    //         }
    //     }
    // }

    // components::internal::RegularSpace rs(
    //     mesh,
    //     pos_handle,
    //     vertex_tag_handle,
    //     edge_tag_handle,
    //     input_tag_value,
    //     embedding_tag_value,
    //     split_tag_value,
    //     1);
    // rs.process();

    // MeshAttributeHandle<long> filter_tag_handle =
    //     mesh.register_attribute<long>("filter_tag", PrimitiveType::Face, 1, false,
    //     input_tag_value);

    // components::internal::Marching mc(
    //     pos_handle,
    //     vertex_tag_handle,
    //     edge_tag_handle,
    //     filter_tag_handle,
    //     input_tag_value,
    //     embedding_tag_value,
    //     split_tag_value);
    // mc.process(mesh);

    // if (true) {
    //     ParaviewWriter writer(data_dir / "mymesh1", "position", mesh, true, true, true, false);
    //     mesh.serialize(writer);
    // }
}
} // namespace components
} // namespace wmtk
