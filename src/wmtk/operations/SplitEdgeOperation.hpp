#pragma once
#include "TupleOperation.hpp"

namespace wmtk {
// TODO: fill out pertinent virtuals
class SplitEdgeOperation : public TupleOperation
{
    SplitEdgeOperation(TriMesh& m, Tuple& t);


    bool execute() override;
    std::vector<Tuple> modified_triangles() const override;

    std::optional<Tuple> return_tuple() const;

    std::optional<std::array<Tuple,2>> original_edge_vertices() const;
    std::optional<Tuple> new_vertex() const;

private:
    std::optional<Tuple> m_return_tuple;
};
} // namespace wmtk
