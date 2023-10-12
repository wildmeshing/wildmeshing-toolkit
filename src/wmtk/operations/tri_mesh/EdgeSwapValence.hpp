
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSwapValence;
}

template <>
struct OperationSettings<tri_mesh::EdgeSwapValence>
{
    bool must_improve_valence = false;
    InvariantCollection invariants;
};

namespace tri_mesh {
class EdgeSwapValence : public TriMeshOperation, private TupleOperation
{
public:
    EdgeSwapValence(Mesh& m, const Tuple& t, const OperationSettings<EdgeSwapValence>& settings);

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_output_tuple;
    const OperationSettings<EdgeSwapValence>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
