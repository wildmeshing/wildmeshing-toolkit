#include "LinkSingleDimensionIterable.hpp"

#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
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
{}

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

    // check if link_type can exist
    {
        const Mesh& mesh = *(m_container->m_mesh);
        const simplex::Simplex& simplex = m_container->m_simplex;
        const PrimitiveType link_pt = m_container->m_link_type;
        const int8_t m = mesh.top_cell_dimension();
        const int8_t s = get_primitive_type_id(simplex.primitive_type());
        const int8_t l = get_primitive_type_id(link_pt);
        if (l >= m - s) {
            logger().warn("Trying to retrieve simplices in the link that cannot exist!");
            m_t = Tuple();
            return;
        }
    }

    init();
}

LinkSingleDimensionIterable::Iterator& LinkSingleDimensionIterable::Iterator::operator++()
{
    if (depth() == 3) {
        return step_depth_3();
    }

    ++m_it;
    m_t = navigate_to_link(*m_it);
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
    m_t = navigate_to_link(*m_it);

    if (depth() == 3) {
        const Mesh& mesh = *(m_container->m_mesh);
        const PrimitiveType& link_type = m_container->m_link_type;

        m_container->m_visited_link.is_visited(mesh.get_id_simplex(m_t, link_type));
    }
}

LinkSingleDimensionIterable::Iterator& LinkSingleDimensionIterable::Iterator::step_depth_3()
{
    const Mesh& mesh = *(m_container->m_mesh);
    const simplex::Simplex& simplex = m_container->m_simplex;
    const PrimitiveType& link_type = m_container->m_link_type;

    if (!is_link_d1()) {
        // iterate for cofaces
        while (!(*m_it).is_null()) {
            for (const Tuple& t : cofaces_in_simplex_iterable(
                     mesh,
                     simplex::Simplex(mesh, simplex.primitive_type(), *m_it),
                     mesh.top_simplex_type())) {
                const Tuple link_tuple = navigate_to_link(t);

                if (!m_container->m_visited_link.is_visited(
                        mesh.get_id_simplex(link_tuple, link_type))) {
                    *m_it = t;
                    m_t = link_tuple;
                    return *this;
                }
            }
            ++m_it;
        }
    } else {
        ++m_it;
    }

    m_t = navigate_to_link(*m_it);
    return *this;
}

Tuple LinkSingleDimensionIterable::Iterator::navigate_to_link(Tuple t)
{
    if (t.is_null()) {
        return t;
    }
    // invert the simplex using SimplexDart
    const Mesh& mesh = *(m_container->m_mesh);
    // const PrimitiveType& mesh_pt = mesh.top_simplex_type();
    // autogen::SimplexDart sd(mesh_pt);

    // switch (mesh.top_simplex_type()) {
    // case PrimitiveType::Triangle: {
    //     const int8_t index_switch = sd.product(
    //         sd.primitive_as_index(PrimitiveType::Edge),
    //         sd.primitive_as_index(PrimitiveType::Vertex));
    //     m_t = autogen::local_switch_tuple(mesh_pt, m_t, index_switch);
    //     break;
    // }
    // default: log_and_throw_error("missing mesh navigation in link"); break;
    // }

    {
        /*
         * Assume a tuple that contains the vertices (a,b,c,d) and the simplex is an edge, i.e.,
         * (a,b). The link contains all the vertices that are not in the simplex. To get a tuple
         * that represents all simplices of the link, we need to move (a,b) to the end of that
         * tuple.
         * (a,b,c,d) becomes (c,d,a,b) with the following permutations
         *              (a,b,c,d)
         * switch edge: (a,c,b,d)
         * switch face: (a,c,d,b)
         * switch vert: (c,a,d,b)
         * switch edge: (c,d,a,b)
         *
         * The following code implements these permutations.
         */
        const simplex::Simplex& simplex = m_container->m_simplex;
        const PrimitiveType link_pt = m_container->m_link_type;
        const int8_t m = mesh.top_cell_dimension();
        const int8_t s = get_primitive_type_id(simplex.primitive_type());

        for (int8_t j = s; j > -1; --j) {
            for (int8_t i = 0; i < m - s; ++i) {
                t = mesh.switch_tuple(t, get_primitive_type_from_id(j + i));
            }
        }
    }

    return t;
}

} // namespace wmtk::simplex
