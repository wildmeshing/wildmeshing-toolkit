#include "multimesh.hpp"

#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/utils/Logger.hpp>
#include "from_boundary.hpp"
#include "from_facet_bijection.hpp"

#include <wmtk/utils/Rational.hpp>


namespace wmtk::components::multimesh {
std::pair<std::shared_ptr<Mesh>, std::shared_ptr<Mesh>> multimesh(
    const MultiMeshType& type,
    Mesh& parent,
    std::shared_ptr<Mesh> child,
    const attribute::MeshAttributeHandle parent_position_handle,
    const std::string& tag_name,
    const int64_t tag_value,
    const int64_t primitive)
{
    if (type == wmtk::components::multimesh::MultiMeshType::UV) {
        if (parent.top_simplex_type() == child->top_simplex_type() &&
            parent.capacity(parent.top_simplex_type()) ==
                child->capacity(child->top_simplex_type())) {
            from_facet_bijection(parent, *child);

            return std::make_pair(parent.shared_from_this(), child);
        } else {
            throw std::runtime_error("unsupported multimesh mapping");
        }
    } else if (
        type == wmtk::components::multimesh::MultiMeshType::Boundary ||
        type == wmtk::components::multimesh::MultiMeshType::Tag) {
        const bool use_boundary = type == wmtk::components::multimesh::MultiMeshType::Boundary;
        std::string tag;
        int64_t value;
        PrimitiveType ptype;

        bool use_rational_position = parent_position_handle.held_type() ==
                                     wmtk::attribute::MeshAttributeHandle::HeldType::Rational;

        if (use_boundary) {
            tag = "is_boundary";
            value = 1;
            ptype = get_primitive_type_from_id(parent.top_cell_dimension() - 1);

            child = from_boundary(parent, ptype, tag, value);
        } else {
            tag = tag_name;
            value = tag_value;
            ptype = get_primitive_type_from_id(primitive);

            child = wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                tag,
                value,
                ptype);
        }


        if (!use_rational_position) {
            auto child_position_handle = child->register_attribute<double>(
                "vertices", // TODO fix me
                PrimitiveType::Vertex,
                parent_position_handle.dimension());

            auto propagate_to_child_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
                return P;
            };
            auto update_child_positon =
                std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                    child_position_handle,
                    parent_position_handle,
                    propagate_to_child_position);
            update_child_positon->run_on_all();
        } else {
            auto child_position_handle = child->register_attribute<Rational>(
                "vertices", // TODO fix me
                PrimitiveType::Vertex,
                parent_position_handle.dimension());

            auto propagate_to_child_position =
                [](const Eigen::MatrixX<Rational>& P) -> Eigen::VectorX<Rational> { return P; };
            auto update_child_positon = std::make_shared<
                wmtk::operations::SingleAttributeTransferStrategy<Rational, Rational>>(
                child_position_handle,
                parent_position_handle,
                propagate_to_child_position);
            update_child_positon->run_on_all();
        }

        return std::make_pair(parent.shared_from_this(), child);
    }

    throw std::runtime_error("unsupported multimesh type");
}

} // namespace wmtk::components::multimesh
