#pragma once
#include <tuple>
#include <wmtk/PrimitiveType.hpp>


namespace wmtk {
class Mesh;
class Tuple;
} // namespace wmtk
namespace wmtk::simplex {

class NavigatableSimplex;

class IdSimplex
{
public:
    friend class wmtk::Mesh;
    friend class NavigatableSimplex;
    IdSimplex() = default;
    IdSimplex(const NavigatableSimplex& s);
    bool valid() const { return m_index == -1; }

    PrimitiveType primitive_type() const { return m_primitive_type; }
    int64_t dimension() const { return get_primitive_type_id(m_primitive_type); }

    bool operator<(const IdSimplex& o) const;
    bool operator==(const IdSimplex& o) const;
    bool operator!=(const IdSimplex& o) const;

    int64_t index() const { return m_index; }

protected:
    IdSimplex(PrimitiveType pt, int64_t index)
        : m_primitive_type(pt)
        , m_index(index)
    {}

    static int64_t id(const Mesh& m, PrimitiveType pt, const Tuple& t);

private:
    PrimitiveType m_primitive_type = PrimitiveType::Vertex;
    int64_t m_index = -1;

private:
    decltype(auto) as_tuple() const { return std::tie(m_primitive_type, m_index); }
};

inline bool IdSimplex::operator<(const IdSimplex& o) const
{
    return as_tuple() < o.as_tuple();
}
inline bool IdSimplex::operator==(const IdSimplex& o) const
{
    return as_tuple() == o.as_tuple();
}
inline bool IdSimplex::operator!=(const IdSimplex& o) const
{
    return as_tuple() != o.as_tuple();
}
} // namespace wmtk::simplex
