#pragma once
#include <wmtk/PrimitiveType.hpp>


namespace wmtk {
class Mesh;
}
namespace wmtk::simplex {

class NavigatableSimplex;

class IdSimplex
{
public:
    friend class wmtk::Mesh;
    IdSimplex() = default;
    IdSimplex(const NavigatableSimplex& s);
    bool valid() const { return m_index == -1; }

    PrimitiveType primitive_type() const { return m_primitive_type; }
    int64_t dimension() const { return get_primitive_type_id(m_primitive_type); }

protected:
    friend class Mesh;
    int64_t index() const { return m_index; }

protected:
    IdSimplex(PrimitiveType pt, int64_t index)
        : m_primitive_type(pt)
        , m_index(index)
    {}
private:

    PrimitiveType m_primitive_type = PrimitiveType::Vertex;
    int64_t m_index = -1;
};
} // namespace wmtk::simplex
