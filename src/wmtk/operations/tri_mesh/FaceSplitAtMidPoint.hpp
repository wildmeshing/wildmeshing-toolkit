#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "FaceSplit.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class FaceSplitAtMidPoint;
}

template <>
struct OperationSettings<tri_mesh::FaceSplitAtMidPoint> : public OperationSettingsBase
{
    OperationSettings<tri_mesh::FaceSplitAtMidPoint>(TriMesh& m)
        : m_mesh(m)
        , split_settings(m)
    {}

    TriMesh& m_mesh;

    OperationSettings<tri_mesh::FaceSplit> split_settings;
    // handle to vertex position
    MeshAttributeHandle<double> position;

    void create_invariants();
};

namespace tri_mesh {
class FaceSplitAtMidPoint : public TriMeshOperation, private TupleOperation
{
public:
    FaceSplitAtMidPoint(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<FaceSplitAtMidPoint>& settings);

    std::string name() const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Face; }

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;
    Accessor<double> m_pos_accessor;

    const OperationSettings<FaceSplitAtMidPoint>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
