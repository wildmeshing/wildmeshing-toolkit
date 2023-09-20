#include "embedded_remeshing.hpp"
#include <wmtk/io/MeshReader.hpp>

#include <igl/read_triangle_mesh.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>

#include "internal/EmbeddedRemeshing.hpp"
#include "internal/EmbeddedRemeshingOptions.hpp"

namespace wmtk {
namespace components {
// this data is the one Prototype displayed
void generateDefaultData(Eigen::MatrixXd& V, Eigen::MatrixXi& E)
{
    V.resize(10, 2);
    E.resize(10, 2);
    V << 3.68892, 4.69909, 3.72799, -2.02109, 3.66695, -1.08875, -1.64113, 3.15912, -1.72872,
        2.57591, 1.20228, 4.31211, 2.19108, -0.595569, 2.78527, -0.139927, -4.85534, 0.28489,
        -4.68596, 3.57936;
    E << 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 0;
}

void embedded_remeshing(
    const nlohmann::json& j,
    std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    EmbeddedRemeshingOptions options = j.get<EmbeddedRemeshingOptions>();

    // some input here !!
    // I thought it could be better if this type can be directly read by libigl.
    // which means we better build a matrix in this block
    TriMesh mesh;
    {
        const std::filesystem::path& file = files[options.input];
        MeshReader reader(file);
        reader.read(mesh);
    }

    // assume we have the data
    Eigen::MatrixXd V;
    Eigen::MatrixXi E;
    generateDefaultData(V, E);

    EmbeddedRemeshing2D embeddedRemeshing2D(E, V);
    embeddedRemeshing2D.process();

    // these are output attributes
    embeddedRemeshing2D.V; // Vertices
    embeddedRemeshing2D.F; // Faces
    embeddedRemeshing2D.Vtags; // flags

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