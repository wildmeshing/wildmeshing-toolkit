
#pragma once
#include <optional>
#include "Operation.hpp"

namespace wmtk {
class TriMeshCollapseEdgeOperation : public Operation
{
public:
    TriMeshCollapseEdgeOperation(Mesh& m, const Tuple& t);

    std::string name() const override;


    std::vector<Tuple> modified_triangles() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_input_tuple;
    Tuple m_output_tuple;
};


} // namespace wmtk

