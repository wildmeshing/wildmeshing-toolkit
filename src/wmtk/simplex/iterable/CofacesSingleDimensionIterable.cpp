#include "CofacesSingleDimensionIterable.hpp"

#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::simplex {


CofacesSingleDimensionIterable::CofacesSingleDimensionIterable(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType cofaces_type)
    : m_mesh(mesh)
    , m_simplex(simplex)
    , m_cofaces_type(cofaces_type)
    , m_tdc_itrbl(mesh, simplex, cofaces_type != mesh.top_simplex_type())
    , m_it_end(m_tdc_itrbl.end())
{
    assert(cofaces_type >= simplex.primitive_type());
}

CofacesSingleDimensionIterable::Iterator::Iterator(
    CofacesSingleDimensionIterable& container,
    const Tuple& t)
    : m_container(container)
    , m_it(container.m_tdc_itrbl, t)
{
    if ((*m_it).is_null()) {
        return;
    }

    init();
}

CofacesSingleDimensionIterable::Iterator& CofacesSingleDimensionIterable::Iterator::operator++()
{
    if (m_container.m_simplex.primitive_type() == m_container.m_cofaces_type) {
        *m_it = Tuple();
        return *this;
    }

    if (depth() == 3) {
        return step_depth_3();
    }

    ++m_it;
    return *this;
}

bool CofacesSingleDimensionIterable::Iterator::operator!=(const Iterator& other) const
{
    return *m_it != *other;
}

Tuple& CofacesSingleDimensionIterable::Iterator::operator*()
{
    return *m_it;
}

const Tuple& CofacesSingleDimensionIterable::Iterator::operator*() const
{
    return *m_it;
}

int64_t CofacesSingleDimensionIterable::Iterator::depth()
{
    const Mesh& mesh = m_container.m_mesh;
    const simplex::Simplex& simplex = m_container.m_simplex;
    assert(mesh.top_cell_dimension() >= get_primitive_type_id(simplex.primitive_type()));
    assert(mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type()) < 4);

    return mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type());
}

bool CofacesSingleDimensionIterable::Iterator::is_coface_d0()
{
    return m_container.m_mesh.top_cell_dimension() ==
           get_primitive_type_id(m_container.m_cofaces_type);
}

void CofacesSingleDimensionIterable::Iterator::init()
{
    if (depth() == 3 && !is_coface_d0()) {
        const Mesh& mesh = m_container.m_mesh;
        const PrimitiveType& cofaces_type = m_container.m_cofaces_type;

        m_container.m_visited_cofaces.is_visited(mesh.get_id_simplex(*m_it, cofaces_type));
    }
}

CofacesSingleDimensionIterable::Iterator& CofacesSingleDimensionIterable::Iterator::step_depth_3()
{
    const Mesh& mesh = m_container.m_mesh;
    const simplex::Simplex& simplex = m_container.m_simplex;
    const PrimitiveType& cofaces_type = m_container.m_cofaces_type;
    auto& visited = m_container.m_visited_cofaces;

    if (!is_coface_d0()) {
        *m_it = mesh.switch_tuples(*m_it, {PrimitiveType::Edge, PrimitiveType::Triangle});
        ++m_edge_counter;

        while (!(*m_it).is_null()) {
            for (; m_edge_counter < 3; ++m_edge_counter) {
                if (!visited.is_visited(mesh.get_id_simplex(*m_it, cofaces_type))) {
                    return *this;
                }
                *m_it = mesh.switch_tuples(*m_it, {PrimitiveType::Edge, PrimitiveType::Triangle});
            }

            // go to next cell
            m_edge_counter = 0;
            ++m_it;
        }
    } else {
        ++m_it;
    }

    return *this;
}

} // namespace wmtk::simplex
