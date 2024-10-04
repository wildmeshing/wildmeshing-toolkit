#include "LinkSingleDimensionIterable.hpp"

#include <wmtk/simplex/cofaces_in_simplex_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>

namespace wmtk::simplex {


LinkSingleDimensionIterable::LinkSingleDimensionIterable(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType link_type)
    : m_mesh(&mesh)
    , m_simplex(simplex)
    , m_link_type(link_type)
    , m_tdc_itrbl(mesh, simplex, mesh.top_cell_dimension() - 1 != get_primitive_type_id(link_type))
    , m_it_end(m_tdc_itrbl.end())
{
    assert(link_type < mesh.top_simplex_type());
}

LinkSingleDimensionIterable::Iterator::Iterator(
    LinkSingleDimensionIterable& container,
    const Tuple& t)
    : m_container(&container)
    , m_it(container.m_tdc_itrbl, t)
    , m_t(t)
{
    if (m_t.is_null()) {
        return;
    }

    init();
}

LinkSingleDimensionIterable::Iterator& LinkSingleDimensionIterable::Iterator::operator++()
{
    if (depth() == 3) {
        return step_depth_3();
    }

    ++m_it;
    m_t = *m_it;
    navigate_to_link();
    return *this;
}

bool LinkSingleDimensionIterable::Iterator::operator!=(const Iterator& other) const
{
    return m_t != *other;
}

Tuple& LinkSingleDimensionIterable::Iterator::operator*()
{
    return m_t;
}

const Tuple& LinkSingleDimensionIterable::Iterator::operator*() const
{
    return m_t;
}

int64_t LinkSingleDimensionIterable::Iterator::depth()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    assert(mesh.top_cell_dimension() >= get_primitive_type_id(simplex.primitive_type()));
    assert(mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type()) < 4);

    return mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type());
}

bool LinkSingleDimensionIterable::Iterator::is_link_d1()
{
    return m_container->m_mesh->top_cell_dimension() - 1 ==
           get_primitive_type_id(m_container->m_link_type);
}

void LinkSingleDimensionIterable::Iterator::init()
{
    log_and_throw_error("missing");
    navigate_to_link();

    if (depth() == 3 && !is_link_d1()) {
        const Mesh& mesh = *(m_container->m_mesh);
        const PrimitiveType& cofaces_type = m_container->m_link_type;

        m_container->m_visited_cofaces.is_visited(mesh.get_id_simplex(*m_it, cofaces_type));
    }
}

LinkSingleDimensionIterable::Iterator& LinkSingleDimensionIterable::Iterator::step_depth_3()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    const PrimitiveType& cofaces_type = m_container->m_link_type;

    log_and_throw_error("step_depth_3 not implemented");

    if (!is_link_d1()) {
        // iterate for cofaces
        while (!(*m_it).is_null()) {
            for (const Tuple& t : cofaces_in_simplex_iterable(
                     mesh,
                     simplex::Simplex(mesh, simplex.primitive_type(), *m_it),
                     mesh.top_simplex_type())) {
                if (!m_container->m_visited_cofaces.is_visited(
                        mesh.get_id_simplex(t, cofaces_type))) {
                    *m_it = t;
                    return *this;
                }
            }
            ++m_it;
        }
    } else {
        ++m_it;
    }

    return *this;
}

void LinkSingleDimensionIterable::Iterator::navigate_to_link()
{
    log_and_throw_error("navigate_to_link not implemented");

    // invert the simplex using SimplexDart
}

} // namespace wmtk::simplex
