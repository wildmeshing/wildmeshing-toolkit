#pragma once

#include <functional>
#include <numeric>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components::internal {

inline long find_index(
    const wmtk::TriMesh& m,
    wmtk::Tuple t,
    std::function<wmtk::Simplex(const wmtk::Tuple&)> extractor,
    wmtk::PrimitiveType type)
{
    auto primitives = m.get_all(type);
    for (int i = 0; i < primitives.size(); ++i) {
        if (m.simplices_are_equal(extractor(primitives[i]), extractor(t))) {
            return i;
        }
    }
    return -1;
}


inline long find_edge_index(const wmtk::TriMesh& m, wmtk::Tuple t)
{
    return find_index(
        m,
        t,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::edge(tuple); },
        wmtk::PrimitiveType::Edge);
}

inline long find_vertex_index(const wmtk::TriMesh& m, wmtk::Tuple t)
{
    return find_index(
        m,
        t,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::vertex(tuple); },
        wmtk::PrimitiveType::Vertex);
}

inline long find_face_index(const wmtk::TriMesh& m, wmtk::Tuple t)
{
    return find_index(
        m,
        t,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::face(tuple); },
        wmtk::PrimitiveType::Face);
}

template <typename Extractor>
inline long connected(
    const wmtk::TriMesh& m,
    wmtk::Simplex i,
    wmtk::Simplex j,
    Extractor extractor,
    wmtk::PrimitiveType type)
{
    auto primitives = m.get_all(type);
    auto i_tuple_list = wmtk::simplex::faces_single_dimension(m, i, type);
    auto j_tuple_list = wmtk::simplex::faces_single_dimension(m, j, type);

    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            if (m.simplices_are_equal(extractor(i_tuple_list[a]), extractor(j_tuple_list[b]))) {
                return find_index(m, i_tuple_list[a], extractor, type);
            }
        }
    }

    return -1;
}

inline long edge_connected(const wmtk::TriMesh& m, wmtk::Simplex i, wmtk::Simplex j)
{
    return connected(
        m,
        i,
        j,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::edge(tuple); },
        wmtk::PrimitiveType::Edge);
}

inline long vertex_connected(const wmtk::TriMesh& m, wmtk::Simplex i, wmtk::Simplex j)
{
    return connected(
        m,
        i,
        j,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::vertex(tuple); },
        wmtk::PrimitiveType::Vertex);
}

wmtk::TriMesh topology_separate_2d(wmtk::TriMesh m);
} // namespace wmtk::components::internal