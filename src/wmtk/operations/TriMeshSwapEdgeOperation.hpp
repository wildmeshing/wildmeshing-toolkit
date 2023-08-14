
#pragma once
#include <optional>
#include "Operation.hpp"

namespace wmtk {
class TriMeshSwapEdgeOperation : public Operation
{
public:
    TriMeshSwapEdgeOperation(
        TriMesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshSwapEdgeOperation> = {});

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_input_tuple;
    Tuple m_output_tuple;
};


} // namespace wmtk
