#pragma once
#include "Simplex.hpp"

namespace wmtk {
class Mesh;
}
namespace wmtk::simplex {
class IdSimplex;
class NavigatableSimplex : public Simplex
{
    friend class IdSimplex;
    friend class wmtk::Mesh;

protected:
    NavigatableSimplex(const PrimitiveType& ptype, const Tuple& t, int64_t index)
        : Simplex{ptype, t}
        , m_index(index)
    {}

private:
    int64_t index() const { return m_index; }
    int64_t m_index;
};
} // namespace wmtk::simplex
