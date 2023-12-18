
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeSwapBase.hpp"
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class ExtremeOptSwap;
}

template <>
struct OperationSettings<tri_mesh::ExtremeOptSwap>
    : public OperationSettings<tri_mesh::EdgeSwapBase>
{
    OperationSettings<tri_mesh::ExtremeOptSwap>(TriMesh& m)
        : OperationSettings<tri_mesh::EdgeSwapBase>(m)
    {}

    // handle to vertex position
    MeshAttributeHandle<double> position;
    std::shared_ptr<TriMesh> uv_mesh_ptr;
    MeshAttributeHandle<double> uv_handle;

    bool optimize_E_max = false;

    void create_invariants();
};

namespace tri_mesh {
/**
 * Perform edge swaps if they improve valence in their neighborhood.
 */
class ExtremeOptSwap : public EdgeSwapBase
{
public:
    ExtremeOptSwap(Mesh& m, const Simplex& t, const OperationSettings<ExtremeOptSwap>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;

private:
    const OperationSettings<ExtremeOptSwap>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
