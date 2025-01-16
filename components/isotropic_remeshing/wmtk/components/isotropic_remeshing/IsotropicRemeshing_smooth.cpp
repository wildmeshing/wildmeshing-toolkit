
#include <memory>
#include <wmtk/components/output/utils/format.hpp>
#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/uvEdgeInvariant.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/composite/ProjectOperation.hpp>
#include <wmtk/operations/utils/VertexLaplacianSmooth.hpp>
#include <wmtk/operations/utils/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk/utils/Logger.hpp>
#include "IsotropicRemeshing.hpp"
#include "invariants/NoChildSimplexInvariant.hpp"
#include <wmtk/components/multimesh/MeshCollection.hpp>


namespace wmtk::components::isotropic_remeshing {
void IsotropicRemeshing::configure_smooth()
{
    auto position = m_options.position_attribute;
    Mesh& mesh = position.mesh();
    auto pass_through_attributes = m_options.pass_through_attributes;
    auto other_positions = m_options.other_position_attributes;
    assert(mesh.is_connectivity_valid());

    std::vector<attribute::MeshAttributeHandle> positions = other_positions;
    positions.push_back(position);
    // if (position.mesh().top_simplex_type() != PrimitiveType::Triangle) {
    //     log_and_throw_error(
    //         "isotropic remeshing works only for triangle meshes: {}",
    //         primitive_type_name(position.mesh().top_simplex_type()));
    // }


    // clear attributes
    std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
    keeps.emplace_back(position);
    keeps.insert(keeps.end(), other_positions.begin(), other_positions.end());

    auto op_smooth = std::make_shared<operations::AttributesUpdateWithFunction>(mesh);

    std::shared_ptr<wmtk::operations::composite::ProjectOperation> proj_op;

    if (position.dimension() == 3 && mesh.top_simplex_type() == PrimitiveType::Triangle) {
        proj_op = std::make_shared<operations::composite::ProjectOperation>(op_smooth);
        proj_op->add_constraint(position, position);
        m_smooth = proj_op;
    } else {
        m_smooth = op_smooth;
    }
    auto update_position_func = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P.col(0);
    };
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        update_position;
    std::optional<attribute::MeshAttributeHandle> position_for_inversion =
        m_options.inversion_position_attribute;

    if (!m_options.other_position_attributes.empty() && !m_options.fix_uv_seam) {
        update_position =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                other_positions.front(),
                position,
                update_position_func);
    }

    if (position.dimension() == 3 && mesh.top_simplex_type() == PrimitiveType::Triangle) {
        op_smooth->set_function(operations::VertexTangentialLaplacianSmooth(position));
    } else {
        op_smooth->set_function(operations::VertexLaplacianSmooth(position));
    }

    if (m_options.lock_boundary) {
        if (!bool(m_interior_position_invariants)) {
            throw std::runtime_error("Interior position invariants were not set despite boundary "
                                     "locking being requested");
        }
        op_smooth->add_invariant(m_interior_position_invariants);
    }

    // hack for uv
    if (m_options.fix_uv_seam) {
        proj_op->add_invariant(
            std::make_shared<wmtk::invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }
    if (m_options.mesh_collection != nullptr) {
        for (const auto& mesh_name : m_options.static_mesh_names) {
            auto& mesh2 = m_options.mesh_collection->get_mesh(mesh_name);
            op_smooth->add_invariant(std::make_shared<invariants::NoChildSimplexInvariant>(mesh,mesh2));

        }
    }

    if (position_for_inversion) {
        proj_op->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }
    // if (m_envelope_invariants) {
    //     op_smooth->add_invariant(m_envelope_invariants);
    // }

    if (update_position) proj_op->add_transfer_strategy(update_position);
}
} // namespace wmtk::components::isotropic_remeshing
