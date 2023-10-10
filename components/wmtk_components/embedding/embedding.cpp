#include "embedding.hpp"
#include <wmtk/io/MeshReader.hpp>

#include <igl/edge_topology.h>
#include <igl/read_triangle_mesh.h>
#include <spdlog/spdlog.h>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/EdgeMeshReader.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <paraviewo/HDF5VTUWriter.hpp>
#include "internal/Embedding.hpp"
#include "internal/EmbeddingOptions.hpp"

namespace wmtk {
namespace components {
// this data is the one Prototype displayed

void embedding(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    EmbeddingOptions options = j.get<EmbeddingOptions>();

    Eigen::MatrixXd vertices_, vertices;
    Eigen::Matrix<long, -1, -1> edges;
    Eigen::MatrixXd W;
    Eigen::MatrixXd VT;
    Eigen::MatrixXd VN;
    Eigen::MatrixXd VP;

    // EdgeMeshReader reader();
    EdgeMesh mesh;
    EdgeMeshReader reader(files[options.input_file], EdgeMeshReader::OBJ);
    reader.read(edges, vertices_, W, VT, VN, VP);
    vertices.resize(vertices_.rows(), 2);

    // assume we have the data
    // one thing should keep in mind, in 2D case, igl::triangulate can
    // only operate vertices with type Eigen::MatrixXd(n x 2)
    // it will crash if the vertices have the thrid dimension value.
    // size convert
    for (long i = 0; i < vertices_.rows(); ++i) {
        // spdlog::info("{} {}", vertices_(i, 0), vertices_(i, 1));
        vertices(i, 0) = vertices_(i, 0);
        vertices(i, 1) = vertices_(i, 1);
    }

    spdlog::info("edges:{} vertices:{} ", edges.rows(), vertices.rows());
    Embedding embedding(edges, vertices, options);
    embedding.process();
    spdlog::info("faces:{} vertices:{} ", embedding.m_faces.rows(), embedding.m_vertices.rows());

    // these are output attributes
    // embeddedRemeshing2D.m_vertex_tags; // Flags
    // embeddedRemeshing2D.m_vertices; // Vertices
    // embeddedRemeshing2D.m_faces; // Faces

    // output
    {
        // const std::filesystem::path cache_dir = "cache";
        // const std::filesystem::path cached_mesh_file = cache_dir / (options.output + ".hdf5");
        const std::filesystem::path cached_mesh_file = options.output + ".hdf5";
        // write to the cache
        HDF5Writer writer(cached_mesh_file);

        TriMesh tri_mesh;
        tri_mesh.initialize(embedding.m_faces);
        // tri_mesh.initialize()

        Eigen::Matrix<long, -1, -1> vertex_tags(embedding.m_vertices.rows(), 1);
        for (long i = 0; i < embedding.m_vertices.rows(); i++) {
            vertex_tags(i, 0) = embedding.m_vertex_tags[i];
        }
        mesh_utils::set_matrix_attribute(
            vertex_tags,
            "m_vertex_tags",
            PrimitiveType::Vertex,
            tri_mesh);

        Eigen::Matrix<long, -1, -1> vertex_idx(embedding.m_vertices.rows(), 1);
        for (long i = 0; i < embedding.m_vertices.rows(); i++) {
            vertex_tags(i, 0) = i;
        }
        mesh_utils::set_matrix_attribute(
            vertex_tags,
            "m_vertex_idx",
            PrimitiveType::Vertex,
            tri_mesh);

        Eigen::MatrixXd position(embedding.m_vertices.rows(), 3);
        for (long i = 0; i < embedding.m_vertices.rows(); i++) {
            position(i, 0) = embedding.m_vertices(i, 0);
            position(i, 1) = embedding.m_vertices(i, 1);
            position(i, 2) = 0.0;
        }
        mesh_utils::set_matrix_attribute(position, "position", PrimitiveType::Vertex, tri_mesh);


        auto exist_in_list =
            [](long idx0, long idx1, const std::vector<std::pair<long, long>>& pair_list) {
                for (long i = 0; i < pair_list.size(); i++) {
                    if (pair_list[i].first == idx0 && pair_list[i].second == idx1) {
                        return true;
                    }
                    if (pair_list[i].first == idx1 && pair_list[i].second == idx0) {
                        return true;
                    }
                }
                return false;
            };


        auto temp_edges = tri_mesh.get_all(PrimitiveType::Edge);
        Eigen::Matrix<long, -1, -1> edge_tags(temp_edges.size(), 1);
        mesh_utils::set_matrix_attribute(edge_tags, "m_edge_tags", PrimitiveType::Edge, tri_mesh);
        auto vid_handle =
            tri_mesh.get_attribute_handle<long>("m_vertex_idx", PrimitiveType::Vertex);
        auto vid_accessor = tri_mesh.create_accessor(vid_handle);
        auto edge_tags_handle =
            tri_mesh.get_attribute_handle<long>("m_edge_tags", PrimitiveType::Edge);
        auto edge_tags_accessor = tri_mesh.create_accessor(edge_tags_handle);
        for (const Tuple& e : temp_edges) {
            long vid0 = vid_accessor.const_vector_attribute(e)(0);
            long vid1 = vid_accessor.const_vector_attribute(tri_mesh.switch_vertex(e))(0);
            if (exist_in_list(vid0, vid1, embedding.m_marked_edges)) {
                edge_tags_accessor.vector_attribute(e)(0) = options.input_tag_value;
            } else {
                edge_tags_accessor.vector_attribute(e)(0) = options.embedding_tag_value;
            }
        }
        // auto handle = tri_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
        // auto edges = tri_mesh.get_all(PrimitiveType::Edge);
        // spdlog::info("{}", edges.size());
        tri_mesh.serialize(writer);
        files[options.output] = cached_mesh_file;

        // for test
        // {
        //     Eigen::MatrixXi nice_f(embedding.m_faces.rows(), 3);
        //     for (int i = 0; i < embedding.m_faces.rows(); ++i) {
        //         nice_f(i, 0) = embedding.m_faces(i, 0);
        //         nice_f(i, 1) = embedding.m_faces(i, 1);
        //         nice_f(i, 2) = embedding.m_faces(i, 2);
        //     }
        //     auto temp_vertices = tri_mesh.get_all(PrimitiveType::Vertex);

        //     auto vertex_tag_handle =
        //         tri_mesh.get_attribute_handle<long>("m_vertex_tags", PrimitiveType::Vertex);
        //     auto vertex_tags_accessor = tri_mesh.create_accessor(vertex_tag_handle);

        //     paraviewo::HDF5VTUWriter writer1;
        //     Eigen::MatrixXd vertex_attributes(position.rows(), 1);
        //     // add inflation/offset attributes
        //     long i = 0;
        //     for (const Tuple& tv : temp_vertices) {
        //         vertex_attributes(i++) = vertex_tags_accessor.vector_attribute(tv)(0);
        //     }
        //     writer1.add_field("vertex_attributes", vertex_attributes);
        //     writer1.write_mesh(options.output + "123result.hdf", embedding.m_vertices, nice_f);
        // }

        // for test
        // {
        //     auto temp_edges = tri_mesh.get_all(PrimitiveType::Edge);
        //     auto edge_tag_handle =
        //         tri_mesh.get_attribute_handle<long>("m_edge_tags", PrimitiveType::Edge);
        //     auto edge_tags_accessor = tri_mesh.create_accessor(edge_tag_handle);

        //     paraviewo::HDF5VTUWriter writer_edges;
        //     Eigen::MatrixXd edges_attributes(temp_edges.size(), 1);
        //     Eigen::MatrixXi E(temp_edges.size(), 2);
        //     long i = 0;
        //     for (const Tuple& te : temp_edges) {
        //         long vid0 = vid_accessor.const_vector_attribute(te)(0);
        //         long vid1 = vid_accessor.const_vector_attribute(tri_mesh.switch_vertex(te))(0);
        //         E(i, 0) = vid0;
        //         E(i, 1) = vid1;
        //         if (exist_in_list(vid0, vid1, embedding.m_marked_edges)) {
        //             edges_attributes(i++) = options.input_tag_value;
        //         } else {
        //             edges_attributes(i++) = options.embedding_tag_value;
        //         }
        //     }
        //     writer_edges.add_cell_field("edges_attributes", edges_attributes);
        //     writer_edges.write_mesh(
        //         options.output + "123result_edges.hdf",
        //         embedding.m_vertices,
        //         E);
        // }
    }
}
} // namespace components
} // namespace wmtk