#pragma once
#include <optional>
#include "Operation.hpp"

namespace wmtk {
class TriMeshSplitEdgeOperation : public Operation
{
public:
    TriMeshSplitEdgeOperation(Mesh& m, const Tuple& t);

    std::string name() const override;


    std::vector<Tuple> modified_triangles() const override;

    Tuple new_vertex() const;

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_input_tuple;
    Tuple m_output_tuple;
};


} // namespace wmtk

