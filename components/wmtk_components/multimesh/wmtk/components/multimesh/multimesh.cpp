#include "multimesh.hpp"

#include "MultimeshOptions.hpp"

#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>


namespace wmtk::components {
void multimesh(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    const std::string type = j["type"];


    std::map<std::string, std::vector<int64_t>> names;
    std::shared_ptr<Mesh> parent;
    std::string out_name;

    if (type == "uv") {
        MultimeshUVOptions options = j.get<MultimeshUVOptions>();
        out_name = options.name;
        parent = cache.read_mesh(options.parent);
        std::shared_ptr<Mesh> child = cache.read_mesh(options.child);

        if (parent->top_simplex_type() == child->top_simplex_type() &&
            parent->capacity(parent->top_simplex_type()) ==
                child->capacity(child->top_simplex_type())) {
            auto child_map = multimesh::same_simplex_dimension_bijection(*parent, *child);

            parent->register_child_mesh(child, child_map);
            names[options.child] = child->absolute_multi_mesh_id();
            names[options.parent] = parent->absolute_multi_mesh_id();
        } else {
            throw std::runtime_error("unsupported multimesh mapping");
        }
    } else if (type == "boundary" || type == "tag") {
        const bool use_boundary = type == "boundary";
        std::shared_ptr<Mesh> mesh;
        std::string tag;
        std::string name;
        std::string position;
        int64_t value;
        PrimitiveType ptype;

        if (use_boundary) {
            MultimeshBOptions options = j.get<MultimeshBOptions>();
            name = options.mesh;
            out_name = options.name;
            mesh = cache.read_mesh(options.mesh);
            position = options.position;

            tag = "is_boundary";
            value = 1;
            ptype = get_primitive_type_from_id(mesh->top_cell_dimension() - 1);

            auto is_boundary_handle = mesh->register_attribute<int64_t>(tag, ptype, 1);
            auto is_boundary_accessor = mesh->create_accessor(is_boundary_handle.as<int64_t>());

            for (const auto& t : mesh->get_all(ptype)) {
                is_boundary_accessor.scalar_attribute(t) = mesh->is_boundary(ptype, t) ? value : 0;
            }
        } else {
            MultimeshTOptions options = j.get<MultimeshTOptions>();
            name = options.mesh;
            out_name = options.name;
            position = options.position;
            tag = options.tag;
            value = options.tag_value;
            ptype = get_primitive_type_from_id(options.primitive);
        }

        auto child_mesh = wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            *mesh,
            tag,
            value,
            ptype);

        auto pt_attribute = mesh->get_attribute_handle<double>(position, PrimitiveType::Vertex);

        auto child_position_handle = child_mesh->register_attribute<double>(
            position,
            PrimitiveType::Vertex,
            mesh->get_attribute_dimension(pt_attribute.as<double>()));

        auto propagate_to_child_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
            return P;
        };
        auto update_child_positon =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                child_position_handle,
                pt_attribute,
                propagate_to_child_position);
        update_child_positon->run_on_all();

        parent = mesh;
        names[name] = parent->absolute_multi_mesh_id();
        names[name + "_" + tag] = child_mesh->absolute_multi_mesh_id();
    }


    // output
    cache.write_mesh(*parent, out_name, names);
}

} // namespace wmtk::components