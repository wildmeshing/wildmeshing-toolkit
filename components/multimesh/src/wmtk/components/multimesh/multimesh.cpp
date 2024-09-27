#include "multimesh.hpp"

#include "MultimeshOptions.hpp"

#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::components {
void multimesh(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    const std::string type = j["type"];


    std::map<std::string, std::vector<int64_t>> names;
    std::shared_ptr<Mesh> parent;
    std::string out_name;

    if (type == "boundary" || type == "tag") {
        std::shared_ptr<Mesh> mesh_in;
        const bool use_boundary = type == "boundary";
        std::string tag;
        std::string name;
        attribute::MeshAttributeHandle position;
        int64_t value;
        PrimitiveType ptype;

        if (use_boundary) {
            MultimeshBOptions options = j.get<MultimeshBOptions>();
            name = options.mesh;
            out_name = options.name;
            mesh_in = cache.read_mesh(options.mesh);
            auto tmp = utils::get_attributes(cache, *mesh_in, options.position);
            assert(tmp.size() == 1);
            position = tmp.front();
            auto& mesh = position.mesh();

            tag = options.tag_name;
            value = 1;
            ptype = get_primitive_type_from_id(mesh.top_cell_dimension() - 1);

            auto is_boundary_handle = mesh.register_attribute<int64_t>(tag, ptype, 1);
            auto is_boundary_accessor = mesh.create_accessor(is_boundary_handle.as<int64_t>());

            for (const auto& t : mesh.get_all(ptype)) {
                is_boundary_accessor.scalar_attribute(t) = mesh.is_boundary(ptype, t) ? value : 0;
            }

            auto child_mesh = wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                position.mesh(),
                tag,
                value,
                ptype);


            auto child_position_handle = child_mesh->register_attribute<double>(
                "vertices", // TODO fix me
                PrimitiveType::Vertex,
                position.dimension());

            auto propagate_to_child_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
                return P;
            };
            auto update_child_positon =
                std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                    child_position_handle,
                    position,
                    propagate_to_child_position);
            update_child_positon->run_on_all();

            parent = mesh_in;
            if (&position.mesh() != mesh_in.get()) names[name] = parent->absolute_multi_mesh_id();

            names[tag] = child_mesh->absolute_multi_mesh_id();
        }

        // output
        cache.write_mesh(*parent, out_name, names);
    }

} // namespace wmtk::components
