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

#include "internal/Embedding.hpp"
#include "internal/EmbeddingOptions.hpp"

namespace wmtk {
namespace components {
// this data is the one Prototype displayed
bool exist_in_list(long idx0, long idx1, const std::vector<std::pair<long, long>>& pair_list)
{
    for (long i = 0; i < pair_list.size(); i++) {
        if (pair_list[i].first == idx0 && pair_list[i].second == idx1) {
            return true;
        }
        if (pair_list[i].first == idx1 && pair_list[i].second == idx0) {
            return true;
        }
    }
    return false;
}

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
        vertices(i, 0) = vertices_(i, 0);
        vertices(i, 1) = vertices_(i, 1);
    }

    spdlog::info("edges:{} vertices:{} ", edges.rows(), vertices.rows());
    Embedding embedding(edges, vertices, options);
    embedding.process();
    spdlog::info("faces:{} vertices:{} ", embedding.m_faces.rows(), embedding.m_vertices.rows());

    // output
    {
        TriMesh tri_mesh;
        tri_mesh.initialize(embedding.m_faces);

        // set vertex tags
        MeshAttributeHandle<long> vertex_tag_handle = tri_mesh.register_attribute<long>(
            "m_vertex_tags",
            PrimitiveType::Vertex,
            1,
            false,
            options.embedding_tag_value);
        Accessor<long> acc_vertex_tag = tri_mesh.create_accessor<long>(vertex_tag_handle);
        {
            int i = 0;
            for (const Tuple& t : tri_mesh.get_all(PrimitiveType::Vertex)) {
                acc_vertex_tag.scalar_attribute(t) = embedding.m_vertex_tags[i];
                i++;
            }
        }

        // set vertex idx
        MeshAttributeHandle<long> vid_handle =
            tri_mesh.register_attribute<long>("m_vertex_idx", PrimitiveType::Vertex, 1, false, -1);
        Accessor<long> acc_vid = tri_mesh.create_accessor<long>(vid_handle);
        {
            int idx = 0;
            for (const Tuple& t : tri_mesh.get_all(PrimitiveType::Vertex)) {
                acc_vid.scalar_attribute(t) = idx;
                idx++;
            }
        }

        // set vertex position
        Eigen::MatrixXd position(embedding.m_vertices.rows(), 3);
        for (long i = 0; i < embedding.m_vertices.rows(); i++) {
            position(i, 0) = embedding.m_vertices(i, 0);
            position(i, 1) = embedding.m_vertices(i, 1);
            position(i, 2) = 0.0;
        }
        mesh_utils::set_matrix_attribute(position, "position", PrimitiveType::Vertex, tri_mesh);

        // set edge tags
        const std::vector<Tuple>& temp_edges = tri_mesh.get_all(PrimitiveType::Edge);
        MeshAttributeHandle<long> edge_tags_handle = tri_mesh.register_attribute<long>(
            "m_edge_tags",
            PrimitiveType::Edge,
            1,
            false,
            options.embedding_tag_value);
        Accessor<long> acc_edge_tags = tri_mesh.create_accessor<long>(edge_tags_handle);
        for (const Tuple& e : temp_edges) {
            long vid0 = acc_vid.const_scalar_attribute(e);
            long vid1 = acc_vid.const_scalar_attribute(tri_mesh.switch_vertex(e));
            if (exist_in_list(vid0, vid1, embedding.m_marked_edges)) {
                acc_edge_tags.scalar_attribute(e) = options.input_tag_value;
            } else {
                acc_edge_tags.scalar_attribute(e) = options.embedding_tag_value;
            }
        }
        // auto handle = tri_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
        // auto edges = tri_mesh.get_all(PrimitiveType::Edge);
        // spdlog::info("{}", edges.size());
        const std::filesystem::path cached_mesh_file = options.output + ".hdf5";
        // write to the cache
        HDF5Writer writer(cached_mesh_file);
        tri_mesh.serialize(writer);
        files[options.output] = cached_mesh_file;

        if (false) {
            ParaviewWriter writer1(cached_mesh_file, "position", tri_mesh, true, true, true, false);
            tri_mesh.serialize(writer1);
        }
    }
}
} // namespace components
} // namespace wmtk