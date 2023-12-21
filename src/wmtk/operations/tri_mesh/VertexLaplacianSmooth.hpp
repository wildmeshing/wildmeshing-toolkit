#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/AttributesUpdateBase.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class VertexLaplacianSmooth;
}

template <>
struct OperationSettings<tri_mesh::VertexLaplacianSmooth>
    : public OperationSettings<AttributesUpdateBase>
{
    OperationSettings<tri_mesh::VertexLaplacianSmooth>(TriMesh& m);

    MeshAttributeHandle<double> position;
    bool smooth_boundary = false;

    void create_invariants();
};

namespace tri_mesh {
class VertexLaplacianSmooth : public AttributesUpdateBase
{
public:
    VertexLaplacianSmooth(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<VertexLaplacianSmooth>& settings);

    std::string name() const override;

protected:
    bool execute() override;

protected:
    Accessor<double> m_pos_accessor;
    const OperationSettings<VertexLaplacianSmooth>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
