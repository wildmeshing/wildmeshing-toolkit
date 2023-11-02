
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeSwapBase.hpp"
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSwapValence;
}

template <>
struct OperationSettings<tri_mesh::EdgeSwapValence>
{
    OperationSettings<tri_mesh::EdgeSwapBase> base_settings;
};

namespace tri_mesh {
/**
 * Perform edge swaps if they improve valence in their neighborhood.
 */
class EdgeSwapValence : public EdgeSwapBase
{
public:
    EdgeSwapValence(Mesh& m, const Tuple& t, const OperationSettings<EdgeSwapValence>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;

private:
    // const OperationSettings<EdgeSwapValence>& m_settings;// TODO unused variable
};

} // namespace tri_mesh
} // namespace wmtk::operations
