#include "LinkIterable.hpp"

#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include <wmtk/simplex/cofaces_in_simplex_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>

namespace wmtk::simplex {


LinkIterable::LinkIterable(const Mesh& mesh, const Simplex& simplex)
    : m_mesh(mesh)
    , m_simplex(simplex)
    , m_tdc_itrbl(mesh, simplex, true)
    , m_it_end(m_tdc_itrbl.end())
{}

LinkIterable::Iterator::Iterator(LinkIterable& container, const Tuple& t)
    : m_container(container)
    , m_it(container.m_tdc_itrbl, t)
    , m_t(t)
{
    if (m_t.is_null()) {
        return;
    }

    // check if link exists
    {
        const int8_t m = m_container.m_mesh.top_cell_dimension();
        const int8_t s = get_primitive_type_id(m_container.m_simplex.primitive_type());
        if (m <= s) {
            logger().warn("Trying to retrieve link that cannot exist!");
            m_t = Tuple();
            return;
        }
    }

    init();
}

LinkIterable::Iterator& LinkIterable::Iterator::operator++()
{
    if (depth() == 3) {
        return step_depth_3();
    }

    const Mesh& mesh = m_container.m_mesh;
    const simplex::Simplex& simplex = m_container.m_simplex;
    const int8_t m = mesh.top_cell_dimension();
    const int8_t s = get_primitive_type_id(simplex.primitive_type());

    if (m_pt < m - s - 1 && !m_it.is_intermediate()) {
        // go to next primitive type
        ++m_pt;
    } else {
        m_pt = 0;
        // change tuple
        ++m_it;
        m_t = navigate_to_link(*m_it);
    }

    return *this;
}

bool LinkIterable::Iterator::operator!=(const Iterator& other) const
{
    return (m_t != other.m_t) || (m_pt != other.m_pt);
}

IdSimplex LinkIterable::Iterator::operator*()
{
    return m_container.m_mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt));
}

const IdSimplex LinkIterable::Iterator::operator*() const
{
    return m_container.m_mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt));
}

int64_t LinkIterable::Iterator::depth()
{
    const Mesh& mesh = m_container.m_mesh;
    const simplex::Simplex& simplex = m_container.m_simplex;
    assert(mesh.top_cell_dimension() >= get_primitive_type_id(simplex.primitive_type()));
    assert(mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type()) < 4);

    return mesh.top_cell_dimension() - get_primitive_type_id(simplex.primitive_type());
}

void LinkIterable::Iterator::init()
{
    m_t = navigate_to_link(*m_it);

    if (depth() == 3) {
        const Mesh& mesh = m_container.m_mesh;

        m_container.m_visited_link[m_pt].is_visited(
            mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt)));
    }
}

LinkIterable::Iterator& LinkIterable::Iterator::step_depth_3()
{
    const Mesh& mesh = m_container.m_mesh;
    const simplex::Simplex& simplex = m_container.m_simplex;
    auto& visited = m_container.m_visited_link;

    ++m_pt;

    if (m_pt == 3) {
        // go to next cell
        m_pt = 0;
        m_edge_counter = 0;
        ++m_it;
        m_t = navigate_to_link(*m_it);
        if (m_t.is_null()) {
            return *this;
        }
    }

    for (; m_edge_counter < 3; ++m_edge_counter) {
        for (; m_pt < 2; ++m_pt) {
            if (!visited[m_pt].is_visited(
                    mesh.get_id_simplex(m_t, get_primitive_type_from_id(m_pt)))) {
                return *this;
            }
        }
        m_t = mesh.switch_tuples(m_t, {PrimitiveType::Vertex, PrimitiveType::Edge});
        m_pt = 0;
    }

    // return face
    m_pt = 2;
    return *this;
}

Tuple LinkIterable::Iterator::navigate_to_link(Tuple t)
{
    if (t.is_null()) {
        return t;
    }
    // invert the simplex using SimplexDart
    const Mesh& mesh = m_container.m_mesh;
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
        const simplex::Simplex& simplex = m_container.m_simplex;
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
