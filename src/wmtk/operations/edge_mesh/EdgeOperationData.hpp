#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

namespace wmtk::operations::edge_mesh {
struct EdgeOperationData
{
    std::array<std::vector<long>, 2> simplex_ids_to_delete;
    std::vector<long> cell_ids_to_update_hash;

    Tuple m_operating_tuple;

    Tuple m_output_tuple;

protected:
    bool m_is_self_loop = false;
    // common simplicies
    std::array<long, 2> m_spine_vids; // V_A_id, V_B_id;
    std::array<long, 2> m_neighbor_eids = {-1, -1};
    long m_operating_edge_id;
};
} // namespace wmtk::operations::edge_mesh
