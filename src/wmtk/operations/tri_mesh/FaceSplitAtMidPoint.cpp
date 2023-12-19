#include "FaceSplitAtMidPoint.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {


void OperationSettings<tri_mesh::FaceSplitAtMidPoint>::create_invariants()
{
    split_settings.create_invariants();

    invariants = std::make_shared<InvariantCollection>(m_mesh);
}

namespace tri_mesh {
FaceSplitAtMidPoint::FaceSplitAtMidPoint(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<FaceSplitAtMidPoint>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.split_settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Face);
}
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
        FaceSplit split_op(mesh(), input_simplex(), m_settings.split_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    m_pos_accessor.vector_attribute(m_output_tuple) = (p0 + p1 + p2) / 3.0;

    return true;
}

std::vector<Simplex> FaceSplitAtMidPoint::modified_primitives() const
{
    Simplex v(PrimitiveType::Vertex, m_output_tuple);
    return {v};
}
std::vector<Simplex> FaceSplitAtMidPoint::unmodified_primitives() const
{
    throw std::runtime_error("not implemented");
    return std::vector<Simplex>();
}
} // namespace tri_mesh
} // namespace wmtk::operations
