#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"
#include "VertexAttributesUpdateBase.hpp"
#include "VertexLaplacianSmooth.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class VertexTangentialLaplacianSmooth;
}

template <>
struct OperationSettings<tri_mesh::VertexTangentialLaplacianSmooth>
    : public OperationSettings<tri_mesh::VertexLaplacianSmooth>
{
    OperationSettings<tri_mesh::VertexTangentialLaplacianSmooth>(TriMesh& m)
        : OperationSettings<tri_mesh::VertexLaplacianSmooth>(m)
    {}

    double damping_factor = 1.0;
};

namespace tri_mesh {
class VertexTangentialLaplacianSmooth : public VertexLaplacianSmooth
{
public:
    VertexTangentialLaplacianSmooth(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<VertexTangentialLaplacianSmooth>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool execute() override;

private:
    const OperationSettings<VertexTangentialLaplacianSmooth>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
