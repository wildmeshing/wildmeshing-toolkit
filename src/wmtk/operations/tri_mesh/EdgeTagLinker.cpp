

#include "EdgeTagLinker.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

namespace wmtk::operations {

namespace tri_mesh {

EdgeTagLinker::EdgeTagLinker(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeTagLinker>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
    , m_vertex_tag_accessor(m.create_accessor(settings.vertex_tag))
    , m_edge_tag_accessor(m.create_accessor(settings.edge_tag))
{}

bool EdgeTagLinker::execute()
{
    // move vertex to center of old vertices
    long vt0 = m_vertex_tag_accessor.vector_attribute(input_tuple())(0);
    long vt1 = m_vertex_tag_accessor.vector_attribute(mesh().switch_vertex(input_tuple()))(0);

    if (vt0 == m_settings.offset_tag_value && vt1 == m_settings.offset_tag_value) {
        m_edge_tag_accessor.vector_attribute(input_tuple())(0) = m_settings.offset_tag_value;
    } else if (vt0 == m_settings.input_tag_value && vt1 == m_settings.input_tag_value) {
        m_edge_tag_accessor.vector_attribute(input_tuple())(0) = m_settings.input_tag_value;
    } else {
        m_edge_tag_accessor.vector_attribute(input_tuple())(0) = m_settings.embedding_tag_value;
    }

    return true;
}

std::string EdgeTagLinker::name() const
{
    return "edge_tag_linker";
}

Tuple EdgeTagLinker::return_tuple() const
{
    return m_output_tuple;
}
} // namespace tri_mesh
} // namespace wmtk::operations
