
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

void EdgeOperationData::sort_split_cell_map()
{
    std::sort(m_split_cell_maps.begin(), m_split_cell_maps.end());
}

// assumes the split cell map has been sorted
const std::array<int64_t, 2>& EdgeOperationData::get_split_output_cells(const int64_t& input_cell)
{
    auto it = std::lower_bound(
        m_split_cell_maps.begin(),
        m_split_cell_maps.end(),
        input_cell,
        [&](const std::tuple<int64_t, std::array<int64_t, 2>>& value, const int64_t& cell) -> bool {
            return std::get<0>(value) < cell;
        });

    assert(it != m_split_cell_maps.end());
    return std::get<1>(*it);
}
} // namespace wmtk::operations
