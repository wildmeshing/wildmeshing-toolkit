#include "RawSimplex.hpp"

#include <algorithm>

#include "RawSimplexCollection.hpp"
#include "Simplex.hpp"
#include "faces_single_dimension.hpp"

namespace wmtk::simplex {
RawSimplex::RawSimplex(const Mesh& mesh, const std::vector<Tuple>& vertices)
{
    m_vertices.reserve(vertices.size());

    ConstAccessor<int64_t> hash_accessor = mesh.get_const_cell_hash_accessor();

    for (size_t i = 0; i < vertices.size(); ++i) {
        m_vertices.emplace_back(
            mesh.is_valid(vertices[i], hash_accessor) ? mesh.id(vertices[i], PrimitiveType::Vertex)
                                                      : -1);
    }

    std::sort(m_vertices.begin(), m_vertices.end());
}

RawSimplex::RawSimplex(std::vector<int64_t>&& vertices)
    : m_vertices{std::move(vertices)}
{
    std::sort(m_vertices.begin(), m_vertices.end());
}

RawSimplex::RawSimplex(const Mesh& mesh, const Simplex& simplex)
    : RawSimplex(
          mesh,
          simplex.primitive_type() == PrimitiveType::Vertex
              ? std::vector<Tuple>{simplex.tuple()}
              : faces_single_dimension_tuples(mesh, simplex, PrimitiveType::Vertex))
{}

int64_t RawSimplex::dimension() const
{
    return m_vertices.size() - 1;
}

bool RawSimplex::operator==(const RawSimplex& o) const
{
    return std::equal(
        m_vertices.begin(),
        m_vertices.end(),
        o.m_vertices.begin(),
        o.m_vertices.end());
}

bool RawSimplex::operator<(const RawSimplex& o) const
{
    if (dimension() != o.dimension()) {
        return dimension() < o.dimension();
    }

    return std::lexicographical_compare(
        m_vertices.begin(),
        m_vertices.end(),
        o.m_vertices.begin(),
        o.m_vertices.end());
}

RawSimplex RawSimplex::opposite_face(const int64_t excluded_id)
{
    std::vector<int64_t> face_ids;
    face_ids.reserve(m_vertices.size() - 1);

    for (const int64_t& v : m_vertices) {
        if (v != excluded_id) {
            face_ids.emplace_back(v);
        }
    }

    RawSimplex face(std::move(face_ids));
    assert(face.dimension() == dimension() - 1);

    return face;
}

RawSimplex RawSimplex::opposite_face(const Mesh& mesh, const Tuple& vertex)
{
    ConstAccessor<int64_t> hash_accessor = mesh.get_const_cell_hash_accessor();

    int64_t excluded_id =
        mesh.is_valid(vertex, hash_accessor) ? mesh.id(vertex, PrimitiveType::Vertex) : -1;

    return opposite_face(excluded_id);
}

RawSimplex RawSimplex::opposite_face(const RawSimplex& face)
{
    const auto& s_v = m_vertices;
    const auto& f_v = face.m_vertices;

    assert(f_v.size() <= s_v.size());

    std::vector<int64_t> o_v;
    o_v.reserve(s_v.size() - f_v.size());

    std::set_difference(
        s_v.begin(),
        s_v.end(),
        f_v.begin(),
        f_v.end(),
        std::inserter(o_v, o_v.begin()));

    assert(o_v.size() == s_v.size() - f_v.size());

    return RawSimplex(std::move(o_v));
}

RawSimplexCollection RawSimplex::faces()
{
    const auto& v = m_vertices;

    std::vector<RawSimplex> faces;

    switch (dimension()) {
    case 0: { // simplex is a vertex
        break;
    }
    case 1: { // simplex is an edge
        faces.reserve(2);
        faces.emplace_back(RawSimplex({v[0]}));
        faces.emplace_back(RawSimplex({v[1]}));
        break;
    }
    case 2: { // simplex is a triangle
        faces.reserve(6);
        faces.emplace_back(RawSimplex({v[0]}));
        faces.emplace_back(RawSimplex({v[1]}));
        faces.emplace_back(RawSimplex({v[2]}));
        faces.emplace_back(RawSimplex({v[0], v[1]}));
        faces.emplace_back(RawSimplex({v[0], v[2]}));
        faces.emplace_back(RawSimplex({v[1], v[2]}));
        break;
    }
    case 3: { // simplex is a tetrahedron
        faces.reserve(14);
        faces.emplace_back(RawSimplex({v[0]}));
        faces.emplace_back(RawSimplex({v[1]}));
        faces.emplace_back(RawSimplex({v[2]}));
        faces.emplace_back(RawSimplex({v[3]}));
        faces.emplace_back(RawSimplex({v[0], v[1]}));
        faces.emplace_back(RawSimplex({v[0], v[2]}));
        faces.emplace_back(RawSimplex({v[0], v[3]}));
        faces.emplace_back(RawSimplex({v[1], v[2]}));
        faces.emplace_back(RawSimplex({v[1], v[3]}));
        faces.emplace_back(RawSimplex({v[2], v[3]}));
        faces.emplace_back(RawSimplex({v[0], v[1], v[2]}));
        faces.emplace_back(RawSimplex({v[0], v[1], v[3]}));
        faces.emplace_back(RawSimplex({v[0], v[2], v[3]}));
        faces.emplace_back(RawSimplex({v[1], v[2], v[3]}));
        break;
    }
    default: throw std::runtime_error("Unexpected dimension in RawSimplex."); break;
    }

    return RawSimplexCollection(std::move(faces));
}

} // namespace wmtk::simplex