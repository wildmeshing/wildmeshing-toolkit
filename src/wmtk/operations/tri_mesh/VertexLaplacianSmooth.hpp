#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"
#include "VertexAttributesUpdateBase.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class VertexLaplacianSmooth;
}

template <>
struct OperationSettings<tri_mesh::VertexLaplacianSmooth>
    : public OperationSettings<tri_mesh::VertexAttributesUpdateBase>
{
    OperationSettings<tri_mesh::VertexLaplacianSmooth>(TriMesh& m)
        : OperationSettings<tri_mesh::VertexAttributesUpdateBase>(m)
    {}

    MeshAttributeHandle<double> position;
    bool smooth_boundary = false;

    void create_invariants();
};

namespace tri_mesh {
class VertexLaplacianSmooth : public VertexAttributesUpdateBase
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
