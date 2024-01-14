#include "BoundaryChecker.hpp"
#include <wmtk/Mesh.hpp>
namespace wmtk::multimesh {
bool BoundaryChecker::is_boundary(const Mesh& mesh, const wmtk::simplex::Simplex& simplex) const
{
    const int64_t my_dimension = mesh.top_cell_dimension();
    if (mesh.is_boundary(simplex)) {
        return true;
    } else {
        for (const Mesh* checker_ptr : m_meshes) {
            if (&mesh == checker_ptr) { // slight optimization to avoid checking my own boundary
                continue;
            }
            if (const int64_t checker_dimension = checker_ptr->top_cell_dimension();
                checker_dimension >= my_dimension) {
                auto checker_simplices = mesh.map_tuples(*checker_ptr, simplex);

                for (const Tuple& t : checker_simplices) {
                    if (checker_ptr->is_boundary(simplex.primitive_type(), t)) {
                        return true;
                    }
                }
            } else if (checker_dimension == my_dimension - 1) {
                if (mesh.can_map(*checker_ptr, simplex)) {
                    return true;
                }
            }
        }
    }
    return false;
}
bool BoundaryChecker::is_boundary(
    const Mesh& mesh,
    const wmtk::PrimitiveType pt,
    const wmtk::Tuple& simplex) const
{
    return is_boundary(mesh, wmtk::simplex::Simplex(pt, simplex));
}
void BoundaryChecker::add_mesh(const Mesh& m)
{
    m_meshes.emplace_back(&m);
}

BoundaryChecker BoundaryChecker::for_all_meshes(const Mesh& m)
{
    BoundaryChecker bc;
    bc.add_mesh(m);
    for (const std::shared_ptr<Mesh>& mptr : m.get_multi_mesh_root().get_all_child_meshes()) {
        bc.add_mesh(*mptr);
    }
    return bc;
}
} // namespace wmtk::multimesh
