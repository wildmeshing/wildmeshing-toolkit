#include <wmtk/Mesh.hpp>
#include "coface_preserving_boundary.hpp"

namespace wmtk::simplex {

std::vector<Tuple> coface_preserving_boundary_tuples(
    const Mesh& mesh,
    const Tuple& t,
    PrimitiveType pt,
    PrimtiiveType coface_pt)
{
    std::vector<Tuple> ret;
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;
    if (coface_pt < pt) {
        ret.emplace_back(t);
    }
    switch (pt) {
    case PrimitiveType::Vertex: {
        // vertex does not have a boundary
    } break;
    case PrimitiveType::Edge: {
    } break;
    case PrimitiveType::Triangle: {
        if (coface_pt < PE) {
            ret.emplace_back(m.switch_tuples(t, {PE}));
        }
    } break;
    case PrimitiveType::Tetrahedron: {
        if (coface_pt < PF) {
            ret.emplace_back(m.switch_tuples(t, {PF}));
            if (coface_pt < PE) {
                ret.emplace_back(m.switch_tuples(t, {PE, PF}));
            }
        }
    } break;
    case PrimitiveType::HalfEdge:
    default: assert(false); break;
    }
    return ret;
}
} // namespace wmtk::simplex
