
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSwap;
}

template <>
struct OperationSettings<tri_mesh::EdgeSwap>
{
    bool must_improve_valence = false;
    InvariantCollection invariants;
};

namespace tri_mesh {
class EdgeSwap : public TriMeshOperation, private TupleOperation
{
public:
    EdgeSwap(Mesh& m, const Tuple& t, const OperationSettings<EdgeSwap>& settings);

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_output_tuple;
    const OperationSettings<EdgeSwap>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
