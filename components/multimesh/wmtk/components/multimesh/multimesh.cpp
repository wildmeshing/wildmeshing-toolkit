#include "multimesh.hpp"

#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/utils/Rational.hpp>


namespace wmtk::components {
std::pair<std::shared_ptr<Mesh>, std::shared_ptr<Mesh>> multimesh(
    const std::string type,
    const std::shared_ptr<Mesh>& mesh,
    const std::shared_ptr<Mesh>& child,
    const std::string position_handle_name,
    const std::string tag_name,
    const int64_t tag_value,
    const int64_t primitive)
{
    std::shared_ptr<Mesh> parent;

    if (type == "uv") {
        parent = mesh;

        if (parent->top_simplex_type() == child->top_simplex_type() &&
            parent->capacity(parent->top_simplex_type()) ==
                child->capacity(child->top_simplex_type())) {
            auto child_map = multimesh::same_simplex_dimension_bijection(*parent, *child);

            parent->register_child_mesh(child, child_map);

            return std::make_pair(parent, child);
        } else {
            throw std::runtime_error("unsupported multimesh mapping");
        }
    } else if (type == "boundary" || type == "tag") {
        std::shared_ptr<Mesh> mesh_in;
        const bool use_boundary = type == "boundary";
        std::string tag;
        attribute::MeshAttributeHandle position;
        int64_t value;
        PrimitiveType ptype;

        bool use_rational_position =
            mesh->has_attribute<Rational>(position_handle_name, PrimitiveType::Vertex);

        if (!use_rational_position) {
            position =
                mesh->get_attribute_handle<double>(position_handle_name, PrimitiveType::Vertex);
        } else {
            position =
                mesh->get_attribute_handle<Rational>(position_handle_name, PrimitiveType::Vertex);
        }

        if (use_boundary) {
            tag = "is_boundary";
            value = 1;
            ptype = get_primitive_type_from_id(mesh->top_cell_dimension() - 1);

            auto is_boundary_handle = mesh->register_attribute<int64_t>(tag, ptype, 1);
            auto is_boundary_accessor = mesh->create_accessor(is_boundary_handle.as<int64_t>());

            for (const auto& t : mesh->get_all(ptype)) {
                is_boundary_accessor.scalar_attribute(t) = mesh->is_boundary(ptype, t) ? value : 0;
            }
        } else {
            tag = tag_name;
            value = tag_value;
            ptype = get_primitive_type_from_id(primitive);
        }

        auto child_mesh = wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            *mesh,
            tag,
            value,
            ptype);

        if (!use_rational_position) {
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
        } else {
            auto child_position_handle = child_mesh->register_attribute<Rational>(
                "vertices", // TODO fix me
                PrimitiveType::Vertex,
                position.dimension());

            auto propagate_to_child_position =
                [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorX<Rational> { return P; };
            auto update_child_positon = std::make_shared<
                wmtk::operations::SingleAttributeTransferStrategy<Rational, Rational>>(
                child_position_handle,
                position,
                propagate_to_child_position);
            update_child_positon->run_on_all();
        }

        return std::make_pair(mesh, child_mesh);
    }

    throw std::runtime_error("unsupported multimesh type");
}

} // namespace wmtk::components