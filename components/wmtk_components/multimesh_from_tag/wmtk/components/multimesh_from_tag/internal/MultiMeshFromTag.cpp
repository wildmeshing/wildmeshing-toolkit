#include "MultiMeshFromTag.hpp"

#include <queue>
#include <set>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/tuples_preserving_primitive_types.hpp>

namespace wmtk::components::internal {

MultiMeshFromTag::MultiMeshFromTag(
    Mesh& mesh,
    const attribute::MeshAttributeHandle& tag_handle,
    const int64_t tag_value)
    : m_mesh(mesh)
    , m_tag_handle(tag_handle)
    , m_tag_value(tag_value)
    , m_tag_acc(m_mesh.create_const_accessor<int64_t>(m_tag_handle))
    , m_tag_ptype(tag_handle.primitive_type())
{
    assert(m_tag_ptype == PrimitiveType::Triangle); // start by implementing only the simple
                                                    // 2d triangle substructure case
}

void MultiMeshFromTag::compute_substructure_ids()
{
    // create attributes to store new ids
    auto v_id_handle = m_mesh.register_attribute<int64_t>(
        "multimesh_from_tag_new_v_ids",
        m_mesh.top_simplex_type(),
        6, // TODOfix: this number depends on the mesh and the substructure type but is at max 6.
        false,
        -1);
    auto v_id_acc = m_mesh.create_accessor<int64_t>(v_id_handle);

    auto vid = [&v_id_acc](const Tuple& t) -> int64_t {
        return v_id_acc.const_vector_attribute(t)(t.local_vid());
    };

    auto set_vid = [&v_id_acc](const Tuple& t, const int64_t val) -> void {
        v_id_acc.vector_attribute(t)(t.local_vid()) = val;
    };

    const auto triangles = m_mesh.get_all(PrimitiveType::Triangle);

    // set vertex ids
    int64_t v_counter = 0;
    for (const Tuple& tri_tuple : triangles) {
        if (m_tag_acc.const_scalar_attribute(tri_tuple) != m_tag_value) {
            continue;
        }

        const auto vertex_tuples = simplex::faces_single_dimension_tuples(
            m_mesh,
            simplex::Simplex::face(tri_tuple),
            PrimitiveType::Vertex);

        for (const Tuple& v_tuple : vertex_tuples) {
            if (vid(v_tuple) != -1) {
                continue;
            }

            const std::vector<Tuple> v_region =
                get_connected_region(v_tuple, PrimitiveType::Vertex);

            for (const Tuple& t : v_region) {
                set_vid(t, v_counter);
            }

            //
            ++v_counter;
        }
    }
}

std::vector<Tuple> MultiMeshFromTag::get_connected_region(
    const Tuple& t_in,
    const PrimitiveType ptype)
{
    if (m_tag_ptype == PrimitiveType::Vertex) {
        return {};
    }

    const PrimitiveType connecting_ptype =
        get_primitive_type_from_id(get_primitive_type_id(m_tag_ptype) - 1);

    std::vector<Tuple> connected_region;

    std::set<Tuple> touched_tuples;
    std::queue<Tuple> q;
    q.push(t_in);

    while (!q.empty()) {
        const Tuple t = q.front();
        q.pop();

        {
            // check if cell already exists
            const auto [it, did_insert] = touched_tuples.insert(t);
            if (!did_insert) {
                continue;
            }
        }
        connected_region.emplace_back(t);

        const std::vector<Tuple> pt_intersection =
            simplex::tuples_preserving_primitive_types(m_mesh, t, ptype, m_tag_ptype);


        for (const Tuple& t_version : pt_intersection) {
            const simplex::Simplex face = simplex::Simplex(connecting_ptype, t_version);
            const std::vector<Tuple> face_cofaces =
                simplex::cofaces_single_dimension_tuples(m_mesh, face, m_tag_ptype);

            std::vector<Tuple> face_cofaces_in_substructure;
            for (const Tuple& fcf : face_cofaces) {
                if (m_tag_acc.const_scalar_attribute(fcf) == m_tag_value) {
                    face_cofaces_in_substructure.emplace_back(fcf);
                }
            }

            if (face_cofaces_in_substructure.size() != 2) {
                // the connection through this face is not manifold or a boundary
                continue;
            }

            assert(
                face_cofaces_in_substructure[0] == t_version ||
                face_cofaces_in_substructure[1] == t_version);

            q.push(
                face_cofaces[0] == t_version ? face_cofaces_in_substructure[1]
                                             : face_cofaces_in_substructure[0]);
        }
    }

    return connected_region;
}

} // namespace wmtk::components::internal
