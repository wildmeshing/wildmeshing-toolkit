#pragma once
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>

namespace wmtk::operations::utils {
class ExtractOperationTuplesFunctor
{
public:
    // std::array<Tuple, 2> operator()(const wmtk::operations::point_mesh::EdgeOperationData& t)
    // const
    //{
    //     return std::array<Tuple, 2>{};
    // }

    std::array<Tuple, 2> operator()(
        const wmtk::operations::edge_mesh::EdgeOperationData& t) const noexcept
    {
        return std::array<Tuple, 2>{{t.m_operating_tuple, t.m_output_tuple}};
    }
    std::array<Tuple, 2> operator()(
        const wmtk::operations::tri_mesh::EdgeOperationData& t) const noexcept
    {
        return std::array<Tuple, 2>{{t.m_operating_tuple, t.m_output_tuple}};
    }
    std::array<Tuple, 2> operator()(
        const wmtk::operations::tet_mesh::EdgeOperationData& t) const noexcept
    {
        return std::array<Tuple, 2>{{t.m_operating_tuple, t.m_output_tuple}};
    }
};
}
