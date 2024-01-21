#include "triangle_insertion.hpp"

#include "TriInsOptions.hpp"

#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
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
    std::shared_ptr<Mesh> bg_mesh = cache.read_mesh(options.background);

    if (mesh_in->top_simplex_type() != PrimitiveType::Face)
        log_and_throw_error("triangle_insertion supports only triangle meshes");
    if (bg_mesh->top_simplex_type() != PrimitiveType::Tetrahedron)
        log_and_throw_error("triangle_insertion supports only bg tet meshes");


    wmtk::utils::EigenMatrixWriter writer;
    mesh_in->serialize(writer);


    wmtk::utils::EigenMatrixWriter writer_bg;
    bg_mesh->serialize(writer_bg);

    Eigen::MatrixXd V;
    writer.get_double_matrix(options.input_position, PrimitiveType::Vertex, V);

    Eigen::MatrixX<int64_t> F;
    writer.get_FV_matrix(F);


    Eigen::MatrixXd Vbg;
    writer_bg.get_double_matrix(options.background_position, PrimitiveType::Vertex, Vbg);

    Eigen::MatrixX<int64_t> Fbg;
    writer_bg.get_TV_matrix(Fbg);

    auto [tetmesh, facemesh] =
        utils::generate_raw_tetmesh_with_surface_from_input(V, F, 0.1, Vbg, Fbg);
    auto pt_attribute =
        tetmesh->get_attribute_handle<double>(options.input_position, PrimitiveType::Vertex);
    auto child_position_handle = facemesh->register_attribute<double>(
        options.input_position,
        PrimitiveType::Vertex,
        tetmesh->get_attribute_dimension(pt_attribute.as<double>()));

    auto propagate_to_child_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P;
    };
    auto update_child_positon =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            child_position_handle,
            pt_attribute,
            propagate_to_child_position);
    update_child_positon->run_on_all();

    cache.write_mesh(*tetmesh, options.name);
}

} // namespace wmtk::components