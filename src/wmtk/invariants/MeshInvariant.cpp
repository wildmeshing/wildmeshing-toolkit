#include "MeshInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/primitive_range.hpp>
namespace wmtk {
MeshInvariant::MeshInvariant(const Mesh& mesh)
    : m_mesh(mesh)
{}

MeshInvariant::~MeshInvariant() = default;
const Mesh& MeshInvariant::mesh() const
{
    return m_mesh;
}

bool MeshInvariant::directly_modified_after(const std::vector<Simplex>& simplices) const
{
    simplex::SimplexCollection all_simplices(mesh());


    for (const Simplex& s : simplices) {
        all_simplices.add(simplex::open_star(mesh(), s, false));
    }
    all_simplices.sort_and_clean();

    for (const PrimitiveType pt : wmtk::utils::primitive_below(mesh().top_simplex_type())) {
        std::vector<Tuple> modified_tuples = all_simplices.simplex_vector_tuples(pt);

        if (!after(pt, modified_tuples)) {
            return false;
        }
    }
    return true;
}

} // namespace wmtk
