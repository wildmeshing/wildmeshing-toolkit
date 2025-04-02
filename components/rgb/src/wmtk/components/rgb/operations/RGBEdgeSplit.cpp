#include "RGBEdgeSplit.hpp"
namespace wmtk::components::rgb::operations {

RGBEdgeSplit::RGBEdgeSplit(
    TriMesh& m,
    const attribute::MeshAttributeHandle& t,
    const attribute::MeshAttributeHandle& e)
    : EdgeSplit(m)
    , m_triangle_level_accessor(t.mesh().create_accessor(t.as<int64_t>())
    , m_edge_level_accessor(e.mesh().create_accessor(e.as<int64_t>())
{}
std::vector<simplex::Simplex> RGBEdgeSplit::execute(const simplex::Simplex& simplex)
{
    //
    {
        auto tri_acc = mesh().create_accessor<int64_t, 2>(m_triangle_level_handle);
        auto edge_acc = mesh().create_accessor<int64_t, 2>(m_edge_level_handle);
        auto edge_color_level = edge_acc.const_vector_attribute(simplex.tuple());
        auto my_face_color_level = tri_acc.const_vector_attribute<2>(simplex.tuple());

        Tuple other_face;
        Eigen::Vector2<int64_t> other_face_color_level(-1, -1);
        if (!mesh().is_boundary(simplex)) {
            other_face = mesh().switch_tuple(simplex.tuple(), PrimitiveType::Triangle);
            other_face_color_level = tri_acc.const_vector_attribute(other_face);
        }

        auto split_return = EdgeSplit::execute(simplex);
        if (split_return.empty()) return {};
        assert(split_return.size() == 1);
        // now we do the attributes update
        Tuple split_return_tuple = split_return[0].tuple();
        assert(edge_color_level[0] == 0);
        // edge attributes update:
        //     split rib edge update:
        Tuple rib_edge = mesh().switch_tuple(split_return_tuple, PrimitiveType::Edge);
        rib_edge_new_attribute_based_on_old_face(rib_edge, my_face_color_level);

        if (!other_face.is_null()) {
            Tuple other_rib_edge = mesh().switch_tuples(
                split_return_tuple,
                {PrimitiveType::Triangle, PrimitiveType::Edge});
            rib_edge_new_attribute_based_on_old_face(other_rib_edge, other_face_color_level);
        }

        //     split edge update:
        split_edge_update(split_return_tuple, edge_color_level);

        // face attributes update:
        face_update(split_return_tuple, my_face_color_level);
        if (!other_face.is_null()) {
            face_update(
                mesh().switch_tuple(split_return_tuple, PrimitiveType::Triangle),
                other_face_color_level);
        }

        return split_return;
    }
}

void RGBEdgeSplit::split_edge_update(
    const Tuple& split_return_edge,
    const Eigen::Vector2<int64_t>& old_edge_color_level)
{
    // old edge == 0 green , l → new_edge(0 green, l+1)
    Tuple new_edge0 = split_return_edge;
    Tuple new_edge1 = mesh().switch_tuples(
        new_edge0,
        {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Edge});
    m_edge_rgb_state_accessor.vector_attribute(new_edge0) =
        Eigen::Vector2<int64_t>(0, old_edge_color_level[1] + 1);
    m_edge_rgb_state_accessor.vector_attribute(new_edge1) =
        Eigen::Vector2<int64_t>(0, old_edge_color_level[1] + 1);
}
void RGBEdgeSplit::rib_edge_new_attribute_based_on_old_face(
    const Tuple& rib_edge,
    const Eigen::Vector2<int64_t>& old_face_color_level)
{
    // if old_face color == 0 green → rib_edge (1 red, l)
    //                      1  red → rib_edge (0 green, l+1)
    if (old_face_color_level[0] == 0) {
        m_edge_rgb_state_accessor.vector_attribute(rib_edge) =
            Eigen::Vector2<int64_t>(1, old_face_color_level[1]);
    } else if (old_face_color_level[0] == 1) {
        m_edge_rgb_state_accessor.vector_attribute(rib_edge) =
            Eigen::Vector2<int64_t>(0, old_face_color_level[1] + 1);
    }
}


void RGBEdgeSplit::face_update(
    const Tuple& split_return,
    const Eigen::Vector2<int64_t>& old_face_color_level)
{
    // if old_face ==  0 green , l →2x new faces  (1 red, l)
    //                 1 red ,   l → if ear edge is  0 green, new face (0 green, l+1)
    //                               if ear edge is  1 red, new face (2 blue, l)
    Tuple new_face0 = split_return;
    Tuple new_face1 =
        mesh().switch_tuples(split_return, {PrimitiveType::Edge, PrimitiveType::Triangle});

    if (old_face_color_level[0] == 0) {
        m_triangle_rgb_state_accessor.vector_attribute(new_face0) =
            Eigen::Vector2<int64_t>(1, old_face_color_level[1]);
        m_triangle_rgb_state_accessor.vector_attribute(new_face1) =
            Eigen::Vector2<int64_t>(1, old_face_color_level[1]);
    } else if (old_face_color_level[0] == 1) {
        Tuple ear_edge0 =
            mesh().switch_tuples(split_return, {PrimitiveType::Vertex, PrimitiveType::Edge});
        red_face_update_based_on_ear_edge(new_face0, ear_edge0, old_face_color_level);

        Tuple ear_edge1 = mesh().switch_tuples(
            new_face1,
            {PrimitiveType::Edge, PrimitiveType::Vertex, PrimitiveType::Edge});
        red_face_update_based_on_ear_edge(new_face1, ear_edge1, old_face_color_level);
    }
}

void RGBEdgeSplit::red_face_update_based_on_ear_edge(
    const Tuple& new_face,
    const Tuple& ear_edge,
    const Eigen::Vector2<int64_t>& old_face_color_level)
{
    assert(old_face_color_level[0] == 1);
    Eigen::Vector2<int64_t> ear_edge_color_level =
        m_edge_rgb_state_accessor.vector_attribute(ear_edge);
    if (ear_edge_color_level[0] == 0) {
        m_triangle_rgb_state_accessor.vector_attribute(new_face) =
            Eigen::Vector2<int64_t>(0, old_face_color_level[1] + 1);
    } else if (ear_edge_color_level[0] == 1) {
        m_triangle_rgb_state_accessor.vector_attribute(new_face) =
            Eigen::Vector2<int64_t>(2, old_face_color_level[1]);
    }
}
} // namespace wmtk::components::rgb::operations
