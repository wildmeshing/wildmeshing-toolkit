#include "MultiMeshFromTag.hpp"

#include <queue>
#include <set>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/tuples_preserving_primitive_types.hpp>
#include <wmtk/utils/primitive_range.hpp>

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
    const PrimitiveType top_pt = m_mesh.top_simplex_type();
    const int64_t top_pt_id = get_primitive_type_id(top_pt);

    // create attributes to store new ids
    for (const PrimitiveType pt : utils::primitive_below(m_mesh.top_simplex_type(), true)) {
        const int64_t pt_id = get_primitive_type_id(pt);

        const int64_t n_ids = m_n_local_ids[top_pt_id][pt_id];

        m_new_id_handles.emplace_back(m_mesh.register_attribute<int64_t>(
            std::string("multimesh_from_tag_new_ids_") + std::to_string(pt_id),
            top_pt,
            n_ids,
            false,
            -1));
    }

    build_adjacency();

    auto v_id_acc = m_mesh.create_accessor<int64_t>(m_new_id_handles[0]);

    auto get_id = [](const attribute::Accessor<int64_t>& acc, const Tuple& t) -> int64_t {
        return acc.const_vector_attribute(t)(t.local_vid());
    };

    auto set_id = [](attribute::Accessor<int64_t>& acc, const Tuple& t, const int64_t val) -> void {
        acc.vector_attribute(t)(t.local_vid()) = val;
    };

    const auto triangles = m_mesh.get_all(PrimitiveType::Triangle);

    // set vertex ids
    int64_t v_counter = 0;
    int64_t f_counter = 0;
    for (const Tuple& tri_tuple : triangles) {
        if (m_tag_acc.const_scalar_attribute(tri_tuple) != m_tag_value) {
            continue;
        }

        const auto vertex_tuples = simplex::faces_single_dimension_tuples(
            m_mesh,
            simplex::Simplex::face(tri_tuple),
            PrimitiveType::Vertex);

        for (const Tuple& v_tuple : vertex_tuples) {
            if (get_id(v_id_acc, v_tuple) != -1) {
                continue;
            }

            const std::vector<Tuple> v_region =
                get_connected_region(v_tuple, PrimitiveType::Vertex);

            for (const Tuple& t : v_region) {
                set_id(v_id_acc, t, v_counter);
            }

            ++v_counter;
        }

        ++f_counter;
    }

    // build FV matrix
    Eigen::MatrixX<int64_t> FV;
    FV.resize(f_counter, v_id_acc.dimension());

    f_counter = 0;
    for (const Tuple& tri_tuple : triangles) {
        if (m_tag_acc.const_scalar_attribute(tri_tuple) != m_tag_value) {
            continue;
        }

        const auto vertex_tuples = simplex::faces_single_dimension_tuples(
            m_mesh,
            simplex::Simplex::face(tri_tuple),
            PrimitiveType::Vertex);

        for (size_t i = 0; i < vertex_tuples.size(); ++i) {
            const int64_t vid = get_id(v_id_acc, vertex_tuples[i]);
            assert(vid != -1);
            FV(f_counter, i) = vid;
        }

        ++f_counter;
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

void MultiMeshFromTag::build_adjacency()
{
    if (m_tag_ptype == PrimitiveType::Vertex) {
        // A point mesh does not have any adjacency
        return;
    }

    const int64_t tag_ptype_id = get_primitive_type_id(m_tag_ptype);

    const int64_t connecting_pt_id = tag_ptype_id - 1;
    const PrimitiveType connecting_ptype = get_primitive_type_from_id(connecting_pt_id);

    m_adjacency_handle = m_mesh.register_attribute<int64_t>(
        "multimesh_from_tag_adjacency",
        m_tag_ptype,
        m_n_local_ids[tag_ptype_id][connecting_pt_id],
        false,
        -1);

    auto adj_acc = m_mesh.create_accessor<int64_t>(m_adjacency_handle);

    const auto top_simplices = m_mesh.get_all(m_tag_ptype);

    auto top_simplex_id_acc = m_mesh.create_accessor<int64_t>(m_new_id_handles[tag_ptype_id]);
    assert(top_simplex_id_acc.dimension() == 1);

    // set cell ids
    int64_t cell_counter = 0;
    for (const Tuple& cell_tuple : top_simplices) {
        if (m_tag_acc.const_scalar_attribute(cell_tuple) == m_tag_value) {
            top_simplex_id_acc.scalar_attribute(cell_tuple) = cell_counter++;
        }
    }

    // create adjacency matrix
    for (const Tuple& cell_tuple : top_simplices) {
        if (m_tag_acc.const_scalar_attribute(cell_tuple) != m_tag_value) {
            continue;
        }

        const auto face_tuples = simplex::faces_single_dimension_tuples(
            m_mesh,
            simplex::Simplex(m_tag_ptype, cell_tuple),
            connecting_ptype);

        for (const Tuple& ft : face_tuples) {
            const simplex::Simplex face = simplex::Simplex(connecting_ptype, ft);
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

            assert(face_cofaces_in_substructure[0] == ft || face_cofaces_in_substructure[1] == ft);

            const Tuple neighbor = face_cofaces[0] == ft ? face_cofaces_in_substructure[1]
                                                         : face_cofaces_in_substructure[0];
            // set adjacency
            {
                int64_t local_id = -1;
                switch (connecting_ptype) {
                case PrimitiveType::Vertex: local_id = ft.local_vid(); break;
                case PrimitiveType::Edge: local_id = ft.local_eid(); break;
                case PrimitiveType::Triangle: local_id = ft.local_fid(); break;
                case PrimitiveType::Tetrahedron:
                default:
                    assert(false); // connecting primitive type cannot be a tetrahedron
                    break;
                }

                adj_acc.vector_attribute(ft)[local_id] =
                    top_simplex_id_acc.const_scalar_attribute(neighbor);
            }
        }
    }
}

} // namespace wmtk::components::internal
