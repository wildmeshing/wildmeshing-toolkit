#include "ATOperations.hpp"
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleTextureIntegralAccuracyFunction.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/invariants/BoundarySimplexInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/simplex/Simplex.hpp>
namespace wmtk::components::adaptive_tessellation::operations::internal {
ATOperations::ATOperations(ATData& atdata, double target_edge_length)
    : m_atdata(atdata)
    , m_target_edge_length(target_edge_length)
    , m_edge_length_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_uv_edge_length_handle.as<double>()))
{
    m_ops.clear();
    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    m_edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_uv_edge_length_handle,
            m_atdata.m_uv_handle,
            compute_edge_length);
    // Lambdas for priority
    m_long_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({-m_edge_length_accessor.scalar_attribute(s.tuple())});
    };
    m_short_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({m_edge_length_accessor.scalar_attribute(s.tuple())});
    };
}

void ATOperations::AT_smooth_interior()
{
    auto& uv_mesh = m_atdata.uv_mesh();
    auto& uv_handle = m_atdata.uv_handle();
    // Energy to optimize
    std::shared_ptr<wmtk::function::PerTriangleTextureIntegralAccuracyFunction> accuracy =
        std::make_shared<wmtk::function::PerTriangleTextureIntegralAccuracyFunction>(
            uv_mesh,
            uv_handle,
            m_atdata.images());

    // std::shared_ptr<wmtk::function::TriangleAMIPS> amips =
    //     std::make_shared<wmtk::function::TriangleAMIPS>(m_atdata.uv_mesh(),
    //     m_atdata.uv_handle());
    // for (auto& f : uv_mesh.get_all(PrimitiveType::Face)) {
    //     auto val = accuracy->get_value(Simplex::face(f));
    //     std::cout << " has value " << val << std::endl;
    // }

    // MeshAttributeHandle handle = amips->attribute_handle();
    // assert(handle.is_valid());
    std::shared_ptr<wmtk::function::LocalNeighborsSumFunction> energy =
        std::make_shared<wmtk::function::LocalNeighborsSumFunction>(
            m_atdata.uv_mesh(),
            m_atdata.uv_handle(),
            *accuracy);
    for (auto& v : uv_mesh.get_all(PrimitiveType::Vertex)) {
        energy->get_value(Simplex::vertex(v));
        break;
    }
    m_ops.emplace_back(std::make_shared<wmtk::operations::OptimizationSmoothing>(energy));
    m_ops.back()->add_invariant(std::make_shared<SimplexInversionInvariant>(uv_mesh, uv_handle));
    m_ops.back()->add_invariant(std::make_shared<InteriorVertexInvariant>(uv_mesh));
    m_ops.back()->add_transfer_strategy(m_edge_length_update);
    m_ops.back()->use_random_priority() = true;
}
void ATOperations::AT_split_interior()
{
    auto& uv_mesh = m_atdata.uv_mesh();
    auto& uv_handle = m_atdata.uv_handle();

    // 1) EdgeSplit
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(uv_mesh);
    split->add_invariant(std::make_shared<TodoLargerInvariant>(
        uv_mesh,
        m_atdata.m_uv_edge_length_handle,
        4.0 / 3.0 * m_target_edge_length));
    // split->add_invariant(std::make_shared<InteriorEdgeInvariant>(uv_mesh));
    split->set_priority(m_long_edges_first);

    split->set_new_attribute_strategy(m_atdata.m_uv_edge_length_handle);
    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_atdata.m_uv_handle);

    split->add_transfer_strategy(m_edge_length_update);
    m_ops.emplace_back(split);
}
void ATOperations::AT_split_single_edge_mesh(Mesh* edge_meshi_ptr)
{
    auto m_t_handle = edge_meshi_ptr->get_attribute_handle<double>("t", PrimitiveType::Vertex);
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*edge_meshi_ptr);
    split->add_invariant(std::make_shared<TodoLargerInvariant>(
        m_atdata.uv_mesh(),
        m_atdata.m_uv_edge_length_handle,
        4.0 / 3.0 * m_target_edge_length));
    split->set_priority(m_long_edges_first);

    split->set_new_attribute_strategy(m_atdata.m_uv_edge_length_handle);
    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_t_handle);

    split->add_transfer_strategy(m_edge_length_update);
    m_ops.emplace_back(split);
}

void ATOperations::AT_split_boundary()
{
    auto& uv_mesh = m_atdata.uv_mesh();
    auto& uv_handle = m_atdata.uv_handle();
    int64_t num_edge_meshes = m_atdata.num_edge_meshes();

    // 1) EdgeSplit on boundary
    for (int64_t i = 0; i < num_edge_meshes; ++i) {
        std::shared_ptr<Mesh> edge_meshi_ptr = m_atdata.edge_mesh_i_ptr(i);
        Mesh* sibling_mesh_ptr = m_atdata.sibling_edge_mesh_ptr(edge_meshi_ptr.get());
        AT_split_single_edge_mesh(edge_meshi_ptr.get());
        if (sibling_mesh_ptr == nullptr) {
            continue;
        }
        AT_split_single_edge_mesh(sibling_mesh_ptr);
    }
}
} // namespace wmtk::components::adaptive_tessellation::operations::internal