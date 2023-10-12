
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSwapBase;
}

template <>
struct OperationSettings<tri_mesh::EdgeSwapBase>
{
    InvariantCollection invariants;
    void initialize_invariants(const TriMesh& m);
};

namespace tri_mesh {
class EdgeSwapBase : public TriMeshOperation, private TupleOperation
{
public:
    EdgeSwapBase(Mesh& m, const Tuple& t, const OperationSettings<EdgeSwapBase>& settings);

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_output_tuple;
    const OperationSettings<EdgeSwapBase>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
