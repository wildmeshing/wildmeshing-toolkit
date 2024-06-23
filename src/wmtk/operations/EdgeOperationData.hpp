#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
}
namespace wmtk::operations {
class EdgeOperationData
{
public:
    Tuple m_operating_tuple;

    Tuple m_output_tuple;
    std::array<int64_t, 2> m_spine_vids; // two endpoints of the edge


    std::vector<std::tuple<int64_t, std::array<int64_t, 2>>> m_split_cell_maps;

    void sort_split_cell_map();

    // assumes the split cell map has been sorted
    const std::array<int64_t, 2>& get_split_output_cells(const int64_t& input_cell);

    std::vector<std::vector<Tuple>> split_boundary_complex;

    // for multimesh we need to know which global ids are modified to trigger
    // for every simplex dimension (We have 3 in trimesh):
    // a list of [simplex index, {all versions of that simplex}]
    std::vector<std::vector<std::tuple<int64_t, std::vector<Tuple>>>>
        global_ids_to_potential_tuples;

protected:
    static Tuple tuple_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid);
};
} // namespace wmtk::operations
