
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

#include <wmtk/Mesh.hpp>
#include "EdgeOperationData.hpp"
namespace wmtk::operations {
Tuple EdgeOperationData::tuple_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid)
{
    return m.tuple_from_id(type, gid);
}

} // namespace wmtk::operations
