
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

#include <wmtk/Mesh.hpp>
#include "EdgeOperationData.hpp"
#include "internal/CollapseAlternateFacetData.hpp"
#include "internal/SplitAlternateFacetData.hpp"
namespace wmtk::operations {

EdgeOperationData::EdgeOperationData() = default;
EdgeOperationData::~EdgeOperationData() = default;
auto EdgeOperationData::tuple_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid)
    -> Tuple
{
    return m.tuple_from_id(type, gid);
}


} // namespace wmtk::operations
