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
{
    OperationSettings<tri_mesh::VertexAttributesUpdateBase> base_settings;
    MeshAttributeHandle<double> position;
    bool smooth_boundary = false;
    void initialize_invariants(const TriMesh& m);
};

namespace tri_mesh {
class VertexLaplacianSmooth : public VertexAttributesUpdateBase
{
public:
    VertexLaplacianSmooth(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexLaplacianSmooth>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool execute() override;

protected:
    Accessor<double> m_pos_accessor;
    const OperationSettings<VertexLaplacianSmooth>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
