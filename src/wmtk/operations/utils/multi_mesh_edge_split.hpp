#pragma once
#include <memory>
#include <wmtk/multimesh/operations/SplitReturnData.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>

namespace wmtk {
class Mesh;
class Tuple;

namespace invariants {
class InvariantCollection;
}
namespace operations::utils {


// Initializes any invariants for splitting (which is None by default, but enabling a pattern
// with other operations)
// TODO: it seems like this is never used?
std::shared_ptr<invariants::InvariantCollection> multimesh_edge_split_invariants(const Mesh& m);

using SplitReturnData = wmtk::multimesh::operations::SplitReturnData;
SplitReturnData multi_mesh_edge_split(
    Mesh& mesh,
    const Tuple& t,
    const std::vector<std::shared_ptr<operations::BaseSplitNewAttributeStrategy>>&
        new_attr_strategies);


} // namespace operations::utils
} // namespace wmtk
