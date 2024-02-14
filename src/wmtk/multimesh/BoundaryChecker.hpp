#pragma once
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Mesh.hpp>
#include <vector>

namespace wmtk {
class Mesh;
class Tuple;
namespace simplex {
class Simplex;
}
} // namespace wmtk
namespace wmtk::multimesh {
    // checks whether a simplex is on the boundary using multimesh
    // youo must manually add meshes as you want to check for boundaryness (and ignore others)
    // the is_boundary is determined in two ways:
    // when compared to a mesh of equal or higher dimension this checks if the chosen simplex is on the boundary of that mesh
    // when compared to a mesh of lesser dimension then it checks if the simplex is mappable at all
class BoundaryChecker
{
public:
    BoundaryChecker() = default;
    BoundaryChecker(const BoundaryChecker&) = default;
    BoundaryChecker(BoundaryChecker&&) = default;
    BoundaryChecker& operator=(const BoundaryChecker&) = default;
    BoundaryChecker& operator=(BoundaryChecker&&) = default;

    static BoundaryChecker for_all_meshes(const Mesh& m);

    template <typename... Args>
    BoundaryChecker(const Mesh& m, Args&&... args);
    void add_mesh(const Mesh& m);

    bool is_boundary(const Mesh& m, const wmtk::simplex::Simplex& simplex) const;
    bool is_boundary(const Mesh& m, const wmtk::PrimitiveType pt, const wmtk::Tuple& simplex) const;

private:
    std::vector<const Mesh*> m_meshes;
};

template <typename... Args>
BoundaryChecker::BoundaryChecker(const Mesh& m, Args&&... rem)
    : BoundaryChecker(std::forward<Args>(rem)...)
{
    m_meshes.emplace_back(&m);
}
} // namespace wmtk::multimesh
