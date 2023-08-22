
#pragma once
#include <optional>
#include "Operation.hpp"

namespace wmtk::operations {
class TriMeshEdgeSwap;

template <>
struct OperationSettings<TriMeshEdgeSwap>
{
    bool must_improve_valence = false;
};


class TriMeshEdgeSwap : public Operation
{
public:
    TriMeshEdgeSwap(
        wmtk::Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshEdgeSwap>& settings = {});

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_input_tuple;
    Tuple m_output_tuple;
    const OperationSettings<TriMeshEdgeSwap>& m_settings;
};


} // namespace wmtk::operations
