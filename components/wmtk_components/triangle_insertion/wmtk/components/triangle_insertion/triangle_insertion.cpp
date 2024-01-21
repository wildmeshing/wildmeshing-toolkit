#include "triangle_insertion.hpp"

#include "TriInsOptions.hpp"

#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/VolumeRemesherTriangleInsertion.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components {

void triangle_insertion(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    TriInsOptions options = j.get<TriInsOptions>();

    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    if (mesh_in->top_simplex_type() != PrimitiveType::Face)
        log_and_throw_error("triangle_insertion supports only triangle meshes");


    wmtk::utils::EigenMatrixWriter writer;
    mesh_in->serialize(writer);

    Eigen::MatrixXd V;
    writer.get_double_matrix(options.position, PrimitiveType::Vertex, V);

    Eigen::MatrixX<int64_t> F;
    writer.get_FV_matrix(F);

    auto [tetmesh, facemesh] = utils::generate_raw_tetmesh_with_surface_from_input(V, F, 0.1);

    cache.write_mesh(*tetmesh, options.name);
}

} // namespace wmtk::components