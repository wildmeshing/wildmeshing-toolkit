#include "embedding.hpp"
#include <wmtk/io/MeshReader.hpp>

#include <igl/read_triangle_mesh.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>

#include "internal/Embedding.hpp"
#include "internal/EmbeddingOptions.hpp"

namespace wmtk {
namespace components {
// this data is the one Prototype displayed
void generateDefaultData(Eigen::MatrixXd& vertices, Eigen::MatrixXi& edges)
{
    vertices.resize(10, 2);
    edges.resize(10, 2);
    vertices << 3.68892, 4.69909, 3.72799, -2.02109, 3.66695, -1.08875, -1.64113, 3.15912, -1.72872,
        2.57591, 1.20228, 4.31211, 2.19108, -0.595569, 2.78527, -0.139927, -4.85534, 0.28489,
        -4.68596, 3.57936;
    edges << 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 0;
}

void embedding(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    EmbeddingOptions options = j.get<EmbeddingOptions>();

    if (false) {
        TriMesh mesh;
        {
            const std::filesystem::path& file = files[options.input];
            MeshReader reader(file);
            reader.read(mesh);
        }
    } else {
        spdlog::warn("REMINDER: Currently using the default data. Please modify the "
                     "emebedded_remeshing.cpp to use user self-defined data.");
    }

    // assume we have the data
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi edges;
    // one thing should keep in mind, in 2D case, igl::triangulate can
    // only operate vertices with type Eigen::MatrixXd(n x 2)
    // it will crash if the vertices have the thrid dimension value.
    generateDefaultData(vertices, edges);

    Embedding embedding(edges, vertices, options);
    embedding.process();

    // these are output attributes
    // embeddedRemeshing2D.m_vertex_tags; // Flags
    // embeddedRemeshing2D.m_vertices; // Vertices
    // embeddedRemeshing2D.m_faces; // Faces

    // output
    {
        const std::filesystem::path cache_dir = "cache";
        const std::filesystem::path cached_mesh_file = cache_dir / (options.output + ".hdf5");

        // write to the cache
        HDF5Writer writer(cached_mesh_file);
        // seems not work...
        // writer.write("tags", 1, 1, embeddedRemeshing2D.Vtags);

        files[options.output] = cached_mesh_file;
    }
}
} // namespace components
} // namespace wmtk