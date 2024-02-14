#include "ValenceImprovementInvariant.hpp"

#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/link.hpp>


namespace wmtk::invariants {
ValenceImprovementInvariant::ValenceImprovementInvariant(const Mesh& m)
    : Invariant(m, true, false, false)
{}
bool ValenceImprovementInvariant::before(const simplex::Simplex& simplex) const
{
    const auto [val_before, val_after] = valence_change(mesh(), simplex);

    if (val_after >= val_before) {
        return false;
    }

    return true;
}

std::pair<int64_t, int64_t> ValenceImprovementInvariant::valence_change(
    const Mesh& mesh,
    const simplex::Simplex& simplex)
{
    const Tuple& t = simplex.tuple();

    assert(simplex.primitive_type() == PrimitiveType::Edge);
    if (mesh.is_boundary(simplex)) {
        return std::make_pair(0, 0);
    }
    const simplex::Simplex f0 = simplex::Simplex::face(t);
    const simplex::Simplex f1 =
        simplex::Simplex::face(mesh.switch_tuple(t, PrimitiveType::Triangle));
    const std::vector<Tuple> vertices_t0 =
        simplex::faces_single_dimension_tuples(mesh, f0, PrimitiveType::Vertex);
    const std::vector<Tuple> vertices_t1 =
        simplex::faces_single_dimension_tuples(mesh, f1, PrimitiveType::Vertex);
    const Tuple v0 = vertices_t0[0];
    const Tuple v1 = vertices_t0[1];
    const Tuple v2 = vertices_t0[2];
    const Tuple v3 = vertices_t1[2];

    auto valence = [&mesh](const Tuple& v) {
        return static_cast<int64_t>(simplex::link(mesh, simplex::Simplex::vertex(v))
                                        .simplex_vector(PrimitiveType::Vertex)
                                        .size());
    };

    int64_t val0 = valence(v0);
    int64_t val1 = valence(v1);
    int64_t val2 = valence(v2);
    int64_t val3 = valence(v3);
    if (mesh.is_boundary(PrimitiveType::Vertex, v0)) {
        val0 += 2;
    }
    if (mesh.is_boundary(PrimitiveType::Vertex, v1)) {
        val1 += 2;
    }
    if (mesh.is_boundary(PrimitiveType::Vertex, v2)) {
        val2 += 2;
    }
    if (mesh.is_boundary(PrimitiveType::Vertex, v3)) {
        val3 += 2;
    }

    // formula from: https://github.com/daniel-zint/hpmeshgen/blob/cdfb9163ed92523fcf41a127c8173097e935c0a3/src/HPMeshGen2/TriRemeshing.cpp#L315
    const int64_t val_before = std::max(std::abs(val0 - 6), std::abs(val1 - 6)) +
                               std::max(std::abs(val2 - 6), std::abs(val3 - 6));
    const int64_t val_after = std::max(std::abs(val0 - 7), std::abs(val1 - 7)) +
                              std::max(std::abs(val2 - 5), std::abs(val3 - 5));
    return std::make_pair(val_before, val_after);
}

} // namespace wmtk::invariants
