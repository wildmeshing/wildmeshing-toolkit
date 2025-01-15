#include "BoundaryChecker.hpp"
#include <wmtk/Mesh.hpp>
namespace wmtk::multimesh {
bool BoundaryChecker::is_boundary(const Mesh& mesh, const wmtk::simplex::Simplex& simplex) const
{
    for (const Mesh* checker_ptr : m_meshes) {
        if (check_mesh(mesh, *checker_ptr, simplex)) {
            return true;
        }
    }
    for (const auto& [mah, v] : m_tags) {
        if (check_tag(mesh, mah, v, simplex)) {
            return true;
        }
    }
    return false;
}
bool BoundaryChecker::check_mesh(
    const Mesh& mesh,
    const Mesh& to_check,
    const wmtk::simplex::Simplex& simplex) const
{
    const int64_t my_dimension = mesh.top_cell_dimension();
    if (&mesh == &to_check) {
        if (mesh.is_boundary(simplex)) {
            return true;
        }
    } else if (const int64_t checker_dimension = to_check.top_cell_dimension();
               checker_dimension >= my_dimension) {
        auto checker_simplices = mesh.map_tuples(to_check, simplex);

        for (const Tuple& t : checker_simplices) {
            if (to_check.is_boundary(simplex.primitive_type(), t)) {
                return true;
            }
        }
    } else if (checker_dimension == my_dimension - 1) {
        if (mesh.can_map(to_check, simplex)) {
            return true;
        }
    }
    return false;
}
bool BoundaryChecker::check_tag(
    const Mesh& m,
    const wmtk::attribute::MeshAttributeHandle& mah,
    const wmtk::attribute::MeshAttributeHandle::ValueVariant& value_var,
    const wmtk::simplex::Simplex& s) const
{
    if (!m.can_map(mah.mesh(), s)) {
        return false;
    } else {
        auto checker_simplices = m.map_tuples(mah.mesh(), s);
        return std::visit(
            [&](const auto& handle, const auto& value) noexcept {
                using HT = std::decay_t<decltype(handle)>;
                using VT = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<typename HT::Type, VT>) {
                    auto acc = mah.mesh().create_const_accessor<VT,1>(handle);
                    for (const Tuple& t : checker_simplices) {
                        if (acc.const_scalar_attribute(t) == value) {
                            return true;
                        }
                    }
                } else {
                    //assert(std::is_same_v<typename HT::Type, VT> == true);
                }
                return false;
            },
            mah.handle(),
            value_var);
    }
}


bool BoundaryChecker::is_boundary(
    const Mesh& mesh,
    const wmtk::PrimitiveType pt,
    const wmtk::Tuple& simplex) const
{
    return is_boundary(mesh, wmtk::simplex::Simplex(mesh, pt, simplex));
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
