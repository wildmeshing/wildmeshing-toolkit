#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/operations/EdgeOperationData.hpp>

namespace wmtk::operations::edge_mesh {
class EdgeOperationData : public wmtk::operations::EdgeOperationData
{
public:
    std::array<std::vector<long>, 2> simplex_ids_to_delete;
    std::vector<long> cell_ids_to_update_hash;


    std::array<long, 2> m_split_e = std::array<long, 2>{{-1, -1}};
    long m_split_v;

    std::array<Tuple, 2> input_endpoints(const EdgeMesh& m) const;
    std::array<Tuple, 2> split_output_edges(const EdgeMesh&) const;

    bool m_is_self_loop = false;
    // common simplicies
    std::array<long, 2> m_spine_vids; // V_A_id, V_B_id;
    std::array<long, 2> m_neighbor_eids = {{-1, -1}};
    long m_operating_edge_id;
};
} // namespace wmtk::operations::edge_mesh
