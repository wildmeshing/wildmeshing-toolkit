#include <wmtk/simplex/internal/HomogeneousSimplexEqualFunctor.hpp>
#include <wmtk/simplex/internal/HomogeneousSimplexLessFunctor.hpp>
#include "unique_homogeneous_simplices.hpp"
namespace wmtk::simplex::utils {


std::vector<Tuple>
unique_homogeneous_simplices(const Mesh& m, PrimitiveType pt, const std::vector<Tuple>& tups)
{
    std::vector<Tuple> vec = tups;
    unique_homogeneous_simplices_inline(m, pt, vec);
    return vec;
}
void unique_homogeneous_simplices_inline(const Mesh& m, PrimitiveType pt, std::vector<Tuple>& tups)
{
    auto less = wmtk::simplex::internal::HomogeneousSimplexLessFunctor(m, pt);
    auto equal = wmtk::simplex::internal::HomogeneousSimplexEqualFunctor(m, pt);

    std::sort(tups.begin(), tups.end(), less);

    tups.erase(std::unique(tups.begin(), tups.end(), equal));
}
} // namespace wmtk::simplex::internal
