#pragma once
#include <memory>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/operations/CollapseReturnData.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>

namespace wmtk {
class Mesh;
class Tuple;

namespace invariants {
class InvariantCollection;
}
namespace attribute::update_strategies {
class UpdateStrategyCollection;
}
namespace operations {
class NewAttributeStrategy;
}
namespace operations::utils {


// Initializes any invariants for collapseting (which is None by default, but enabling a pattern
// with other operations)
// TODO: it seems like this is never used?
std::shared_ptr<invariants::InvariantCollection> multimesh_edge_collapse_invariants(const Mesh& m);


using CollapseReturnData = wmtk::multimesh::operations::CollapseReturnData;

CollapseReturnData multi_mesh_edge_collapse(
    Mesh& mesh,
    const Tuple& t,
    const std::vector<std::shared_ptr<operations::BaseCollapseNewAttributeStrategy>>&
        new_attr_strategies);


std::vector<simplex::Simplex> multi_mesh_edge_collapse_with_modified_simplices(
    Mesh& mesh,
    const simplex::Simplex& simplex,
    const std::vector<std::shared_ptr<operations::BaseCollapseNewAttributeStrategy>>&
        new_attr_strategies);

} // namespace operations::utils
} // namespace wmtk
