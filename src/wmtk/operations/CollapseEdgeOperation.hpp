#pragma once
#include "TupleOperation.hpp"

namespace wmtk {
// TODO: fill out pertinent virtuals
class CollapseEdgeOperation : public TupleOperation
{
    CollapseEdgeOperation(TriMesh& m, Tuple& t);


    bool execute() override;
    std::vector<Tuple> modified_triangles() const override;
    std::optional<Tuple> return_tuple() const;

private:
    std::optional<Tuple> m_return_tuple;
};
} // namespace wmtk
