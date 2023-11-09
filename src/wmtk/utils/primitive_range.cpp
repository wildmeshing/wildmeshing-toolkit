#include "primitive_range.hpp"
namespace wmtk::utils {
std::vector<PrimitiveType> primitive_range(PrimitiveType pt0, PrimitiveType pt1)
{
    std::vector<PrimitiveType> r;
    switch (pt0) {
    case PrimitiveType::Vertex:
        r.emplace_back(PrimitiveType::Vertex);
        if (pt1 == r.back()) {
            break;
        }
        [[fallthrough]];
    case PrimitiveType::Edge:
        r.emplace_back(PrimitiveType::Edge);
        if (pt1 == r.back()) {
            break;
        }
        [[fallthrough]];
    case PrimitiveType::Face:
        r.emplace_back(PrimitiveType::Face);
        if (pt1 == r.back()) {
            break;
        }
        [[fallthrough]];
    case PrimitiveType::Tetrahedron:
        r.emplace_back(PrimitiveType::Tetrahedron);
        if (pt1 == r.back()) {
            break;
        }
        [[fallthrough]];
    case PrimitiveType::HalfEdge:
    default: break;
    }
    return r;
}
std::vector<PrimitiveType> primitive_above(PrimitiveType pt)
{
    std::vector<PrimitiveType> r;

    switch (pt) {
    case PrimitiveType::Vertex: r.emplace_back(PrimitiveType::Vertex); [[fallthrough]];
    case PrimitiveType::Edge: r.emplace_back(PrimitiveType::Edge); [[fallthrough]];
    case PrimitiveType::Face: r.emplace_back(PrimitiveType::Face); [[fallthrough]];
    case PrimitiveType::Tetrahedron: r.emplace_back(PrimitiveType::Tetrahedron); [[fallthrough]];
    case PrimitiveType::HalfEdge:
    default: break;
    }
    return r;
}
std::vector<PrimitiveType> primitive_below(PrimitiveType pt)
{
    std::vector<PrimitiveType> r;

    switch (pt) {
    case PrimitiveType::Tetrahedron: r.emplace_back(PrimitiveType::Tetrahedron); [[fallthrough]];
    case PrimitiveType::Face: r.emplace_back(PrimitiveType::Face); [[fallthrough]];
    case PrimitiveType::Edge: r.emplace_back(PrimitiveType::Edge); [[fallthrough]];
    case PrimitiveType::Vertex: r.emplace_back(PrimitiveType::Vertex); [[fallthrough]];
    case PrimitiveType::HalfEdge:
    default: break;
    }
    return r;
}
} // namespace wmtk::utils
