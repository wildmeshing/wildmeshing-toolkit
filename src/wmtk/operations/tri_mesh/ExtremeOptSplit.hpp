#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class ExtremeOptSplit;
}

template <>
struct OperationSettings<tri_mesh::ExtremeOptSplit> : public OperationSettings<tri_mesh::EdgeSplit>
{
    // constructor
    OperationSettings<tri_mesh::ExtremeOptSplit>(TriMesh& m)
        : OperationSettings<tri_mesh::EdgeSplit>(m)
    {}

    // handle to vertex position
    MeshAttributeHandle<double> position;
    // too short edges get ignored
    double min_squared_length = -1;

    std::shared_ptr<TriMesh> uv_mesh_ptr;
    MeshAttributeHandle<double> uv_handle;

    void create_invariants();
};

namespace tri_mesh {
class ExtremeOptSplit : public EdgeSplit
{
public:
    ExtremeOptSplit(Mesh& m, const Simplex& t, const OperationSettings<ExtremeOptSplit>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    std::vector<double> priority() const override;

protected:
    bool before() const override;
    bool execute() override;

private:
    Accessor<double> m_pos_accessor;
    Accessor<double> m_uv_accessor;
    const OperationSettings<ExtremeOptSplit>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
