#pragma once
#include <optional>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
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
{
    OperationSettings<tri_mesh::VertexLaplacianSmooth> smooth_settings;
    double damping_factor = 1.0;
    void initialize_invariants(const TriMesh& m);
};

namespace tri_mesh {
class VertexTangentialLaplacianSmooth : public VertexLaplacianSmooth
{
public:
    VertexTangentialLaplacianSmooth(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexTangentialLaplacianSmooth>& settings);

    std::string name() const override;

protected:
    bool execute() override;

private:
    const OperationSettings<VertexTangentialLaplacianSmooth>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
