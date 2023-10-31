#include "FaceSplitAtMidPoint.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {


void OperationSettings<tri_mesh::FaceSplitAtMidPoint>::initialize_invariants(const TriMesh& m)
{
    split_settings.initialize_invariants(m);
}

bool OperationSettings<tri_mesh::FaceSplitAtMidPoint>::are_invariants_initialized() const
{
    return split_settings.are_invariants_initialized();
}
namespace tri_mesh {
FaceSplitAtMidPoint::FaceSplitAtMidPoint(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<FaceSplitAtMidPoint>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.split_settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{}
std::string FaceSplitAtMidPoint::name() const
{
    return "tri_mesh_split_face_at_midpoint";
}
Tuple FaceSplitAtMidPoint::return_tuple() const
{
    return m_output_tuple;
}
bool FaceSplitAtMidPoint::execute()
{
    const Eigen::Vector3d p0 = m_pos_accessor.vector_attribute(input_tuple());
    const Eigen::Vector3d p1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
    const Eigen::Vector3d p2 =
        m_pos_accessor.vector_attribute(mesh().switch_vertex(mesh().switch_edge(input_tuple())));

    {
        FaceSplit split_op(mesh(), input_tuple(), m_settings.split_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    m_pos_accessor.vector_attribute(m_output_tuple) = (p0 + p1 + p2) / 3.0;

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
