#pragma once
#include <wmtk/Tuple.hpp>
#include "IdSimplex.hpp"
#include "Simplex.hpp"

namespace wmtk {
class Mesh;
}
namespace wmtk::simplex {
class IdSimplex;


// An extension to a simplex that can be used for some sort of navigation at the time it is created.
// Useful for tracking the input of operations where the simplex id is valuable
class NavigatableSimplex : public IdSimplex
{
public:
    friend class wmtk::Mesh;
    NavigatableSimplex(const Mesh& m, const Simplex& s);
    NavigatableSimplex(const Mesh& m, const PrimitiveType& ptype, const Tuple& t) ;
    operator Simplex() const;


    const Tuple& tuple() const { return m_tuple; }

protected:
    NavigatableSimplex(const PrimitiveType& ptype, const Tuple& t, int64_t index)
        : IdSimplex{ptype, index}
        , m_tuple(t)
    {}


private:
    Tuple m_tuple;
};
} // namespace wmtk::simplex
