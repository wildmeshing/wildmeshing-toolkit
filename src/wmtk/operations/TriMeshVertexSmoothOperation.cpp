#include "TriMeshVertexSmoothOperation.hpp"

namespace wmtk {
TriMeshVertexSmoothOperation::TriMeshVertexSmoothOperation(
    Mesh& m,
    const Tuple& t,
    const Settings& settings)
    : Operation(m)
    , m_tuple(t)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
{}

std::string TriMeshVertexSmoothOperation::name() const
{
    return "vertex_smooth";
}

bool TriMeshVertexSmoothOperation::before() const
{
    return true;
}

bool TriMeshVertexSmoothOperation::execute()
{
    return true;
}


} // namespace wmtk