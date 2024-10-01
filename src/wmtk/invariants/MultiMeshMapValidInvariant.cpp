#include "MultiMeshMapValidInvariant.hpp"

#include <stdexcept>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/MultiMeshSimplexVisitor.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>

namespace wmtk {
namespace {

bool are_all_ears_in_child(const TriMesh& parent, const EdgeMesh& child, const Tuple& t)
{
    const Tuple parent_ear_0 = parent.switch_edge(t);
    const Tuple paretn_ear_1 = parent.switch_edge(parent.switch_vertex(t));
    bool find_ear_0 =
        !parent.map_to_child(child, simplex::Simplex::edge(parent, parent_ear_0)).empty();
    bool find_ear_1 =
        !parent.map_to_child(child, simplex::Simplex::edge(parent, paretn_ear_1)).empty();
    return find_ear_0 && find_ear_1;
}

bool are_all_ears_in_child(const TetMesh& parent, const TriMesh& child, const Tuple& t)
{
    const Tuple parent_ear_0 = parent.switch_face(parent.switch_edge(t));
    const Tuple parent_ear_1 = parent.switch_face(parent.switch_edge(parent.switch_vertex(t)));
    bool find_ear_0 =
        !parent.map_to_child(child, simplex::Simplex::face(parent, parent_ear_0)).empty();
    bool find_ear_1 =
        !parent.map_to_child(child, simplex::Simplex::face(parent, parent_ear_1)).empty();
    return find_ear_0 && find_ear_1;
}

bool are_all_ears_in_child(const TetMesh& parent, const EdgeMesh& child, const Tuple& t)
{
    const Tuple parent_ear_0 = parent.switch_edge(t);
    const Tuple parent_ear_1 = parent.switch_edge(parent.switch_vertex(t));
    bool find_ear_0 =
        !parent.map_to_child(child, simplex::Simplex::edge(parent, parent_ear_0)).empty();
    bool find_ear_1 =
        !parent.map_to_child(child, simplex::Simplex::edge(parent, parent_ear_1)).empty();

    const Tuple t_switch_face = parent.switch_face(t);
    const Tuple parent_ear_2 = parent.switch_edge(t_switch_face);
    const Tuple parent_ear_3 = parent.switch_edge(parent.switch_vertex(t_switch_face));
    bool find_ear_2 =
        !parent.map_to_child(child, simplex::Simplex::edge(parent, parent_ear_2)).empty();
    bool find_ear_3 =
        !parent.map_to_child(child, simplex::Simplex::edge(parent, parent_ear_3)).empty();

    return (find_ear_0 && find_ear_1) || (find_ear_2 && find_ear_3);
}
struct MultiMeshMapValidFunctor
{
    bool operator()(const auto& m, const simplex::Simplex& s, int64_t) {
        return this->operator()(m,s);
    }
    bool operator()(const Mesh& m, const simplex::Simplex& s) const { return false; }
    bool operator()(const PointMesh& m, const simplex::Simplex& s) const { return false; }

    bool operator()(const EdgeMesh& m, const simplex::Simplex& s) const { return false; }
    bool operator()(const TriMesh& m, const simplex::Simplex& s) const
    {
        const std::vector<Tuple> equivalent_tuples = simplex::top_dimension_cofaces_tuples(m, s);

        for (auto child_ptr : m.get_child_meshes()) {
            if (child_ptr->top_cell_dimension() != 1) {
                continue;
            }

            for (const Tuple& t : equivalent_tuples) {
                const EdgeMesh& child = dynamic_cast<const EdgeMesh&>(*child_ptr);
                if (m.map_to_child(child, s).empty() && are_all_ears_in_child(m, child, t)) {
                    return false;
                }
            }
        }
        return true;
    }
    bool operator()(const TetMesh& m, const simplex::Simplex& s) const
    {
        const std::vector<Tuple> equivalent_tuples = simplex::top_dimension_cofaces_tuples(m, s);

        for (auto child_ptr : m.get_child_meshes()) {
            if (child_ptr->top_cell_dimension() != 2 && child_ptr->top_cell_dimension() != 1) {
                continue;
            }

            for (const Tuple& t : equivalent_tuples) {
                if (child_ptr->top_cell_dimension() == 2) {
                    const TriMesh& child = dynamic_cast<const TriMesh&>(*child_ptr);
                    if (m.map_to_child(child, s).empty() && are_all_ears_in_child(m, child, t)) {
                        return false;
                    }
                } else {
                    const EdgeMesh& child = dynamic_cast<const EdgeMesh&>(*child_ptr);
                    if (m.map_to_child(child, s).empty() && are_all_ears_in_child(m, child, t)) {
                        return false;
                    }
                }
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
    visitor.execute_from_root(const_cast<Mesh&>(mesh()), simplex::NavigatableSimplex(mesh(),t));
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
