#include "OpenStarIterable.hpp"

#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include <wmtk/simplex/cofaces_in_simplex_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>

namespace wmtk::simplex {


OpenStarIterable::OpenStarIterable(const Mesh& mesh, const Simplex& simplex)
    : m_mesh(&mesh)
    , m_simplex(simplex)
    , m_tdc_itrbl(mesh, simplex, true)
    , m_it_end(m_tdc_itrbl.end())
{}

OpenStarIterable::Iterator::Iterator(OpenStarIterable& container, const Tuple& t)
    : m_container(&container)
    , m_it(container.m_tdc_itrbl, t)
    , m_t(t)
    , m_pt(get_primitive_type_id(container.m_simplex.primitive_type()))
{
    if (m_t.is_null()) {
        return;
    }

    init();
}

OpenStarIterable::Iterator& OpenStarIterable::Iterator::operator++()
{
    if (depth() == 3) {
        return step_depth_3();
    }

    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    const int8_t m = mesh.top_cell_dimension();
    const int8_t s = get_primitive_type_id(simplex.primitive_type());

    if (m_pt < m && !m_it.is_intermediate()) {
        // go to next primitive type
        ++m_pt;
    } else {
        m_pt = s + 1;
        // change tuple
        ++m_it;
        m_t = *m_it;
    }

    if (m_t.is_null()) {
        m_pt = s;
    }

    return *this;
}

bool OpenStarIterable::Iterator::operator!=(const Iterator& other) const
{
    return (m_t != other.m_t) || (m_pt != other.m_pt);
}

IdSimplex OpenStarIterable::Iterator::operator*()
{
    return m_container->m_mesh->get_id_simplex(m_t, get_primitive_type_from_id(m_pt));
}

const IdSimplex OpenStarIterable::Iterator::operator*() const
{
    return m_container->m_mesh->get_id_simplex(m_t, get_primitive_type_from_id(m_pt));
}

int64_t OpenStarIterable::Iterator::depth()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    assert(mesh.top_cell_dimension() >= get_primitive_type_id(simplex.primitive_type()));
    assert(mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type()) < 4);

    return mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type());
}

void OpenStarIterable::Iterator::init() {}

OpenStarIterable::Iterator& OpenStarIterable::Iterator::step_depth_3()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    auto& visited = m_container->m_visited_cofaces;

    assert(mesh.top_simplex_type() == PrimitiveType::Tetrahedron);
    assert(simplex.primitive_type() == PrimitiveType::Vertex);

    ++m_pt;

    if (m_pt == 4) {
        // go to next cell
        m_pt = 1;
        m_edge_counter = 0;
        ++m_it;
        m_t = *m_it;
        if (m_t.is_null()) {
            m_pt = 0;
            return *this;
        }
    }

    for (; m_edge_counter < 3; ++m_edge_counter) {
        for (; m_pt < 3; ++m_pt) {
            if (!visited[m_pt - 1].is_visited(
                    mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt)))) {
                return *this;
            }
        }
        m_t = mesh.switch_tuples(m_t, {PrimitiveType::Edge, PrimitiveType::Triangle});
        m_pt = 1;
    }

    // return tet
    m_pt = 3;
    return *this;
}

} // namespace wmtk::simplex
