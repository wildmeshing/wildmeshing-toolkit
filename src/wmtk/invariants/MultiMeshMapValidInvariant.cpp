#include "MultiMeshMapValidInvariant.hpp"

#include <stdexcept>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/MultiMeshSimplexVisitor.hpp>
#include <wmtk/simplex/cofaces_single_dimension_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include "wmtk/simplex/cofaces_single_dimension.hpp"

namespace wmtk {
namespace {

// checks if two simplices both are mappable
bool both_map_to_child(
    const Mesh& parent,
    const Mesh& child,
    const simplex::Simplex& left,
    const simplex::Simplex& right)
{
    return parent.can_map(child, left) && parent.can_map(child, right);
}


// computes teh two ears in a K+1 simplex over the input edge to see if their facets will be mapped
// into one another
bool both_map_to_child(const Mesh& parent, const Mesh& child, const Tuple& input)
{
    const PrimitiveType child_type = child.top_simplex_type();
    const PrimitiveType collapsed_simplex_type = child_type + 1;
    auto opposite = [&parent, collapsed_simplex_type](Tuple t) {
        for (PrimitiveType pt = collapsed_simplex_type; pt > PrimitiveType::Vertex; pt = pt - 1) {
            t = parent.switch_tuple(t, pt);
        }
        return t;
    };
    const simplex::Simplex left(child_type, opposite(input));
    const simplex::Simplex right(
        child_type,
        opposite(parent.switch_tuple(input, PrimitiveType::Vertex)));
    return both_map_to_child(parent, child, left, right);
}


// two child  K-facets will merge into one another if they are the ears of a K+1 simplex whose
// "input edge" is the input edge. This function iterates through those K+1 simplices and lets
// both_map_to_child check for if both ears are mapped
bool any_pairs_both_map_to_child(
    const Mesh& parent,
    const Mesh& child,
    const simplex::Simplex& edge)
{
    assert(edge.primitive_type() == PrimitiveType::Edge);
    const PrimitiveType parent_type = parent.top_simplex_type();
    const PrimitiveType child_type = child.top_simplex_type();
    assert(parent_type > child_type);
    if (parent_type == child_type + 1) {
        return both_map_to_child(parent, child, edge.tuple());
    }
    for (const Tuple& tuple :
         simplex::cofaces_single_dimension_iterable(parent, edge, child.top_simplex_type() + 1)) {
        if (both_map_to_child(parent, child, tuple)) {
            return true;
        }
    }
    return false;
}


struct MultiMeshMapValidFunctor
{
    template <typename T>
    bool operator()(const T& m, const simplex::Simplex& s, int64_t)
    {
        return this->operator()(m, s);
    }
    bool operator()(const Mesh& m, const simplex::Simplex& s) const
    {
        for (auto child_ptr : m.get_child_meshes()) {
            if (any_pairs_both_map_to_child(m, *child_ptr, s)) {
                return false;
            }
        }
        return true;
    }
};
} // namespace

MultiMeshMapValidInvariant::MultiMeshMapValidInvariant(const Mesh& m)
    : Invariant(m, true, false, false)
{}
bool MultiMeshMapValidInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);
    multimesh::MultiMeshSimplexVisitor visitor(
        std::integral_constant<int64_t, 1>{}, // specify that this runs on edges
        MultiMeshMapValidFunctor{});
    // TODO: fix visitor to work for const data
    visitor.execute_from_root(const_cast<Mesh&>(mesh()), simplex::NavigatableSimplex(mesh(), t));
    const auto& data = visitor.cache();

    for (const auto& [key, value_var] : data) {
        const bool valid = std::get<bool>(value_var);
        if (!valid) {
            return false;
        }
    }
    return true;
}
} // namespace wmtk
