
#pragma once
#include <optional>
#include "Operation.hpp"

namespace wmtk {
class TriMeshSwapEdgeOperation;

template <>
struct OperationSettings<TriMeshSwapEdgeOperation>
{
    bool must_improve_valence = false;
};


class TriMeshSwapEdgeOperation : public Operation
{
public:
    TriMeshSwapEdgeOperation(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshSwapEdgeOperation>& settings = {});

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_input_tuple;
    Tuple m_output_tuple;
    bool m_must_improve_valence;
};


} // namespace wmtk
