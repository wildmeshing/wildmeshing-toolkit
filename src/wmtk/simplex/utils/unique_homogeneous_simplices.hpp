#pragma once
#include <vector>
#include <wmtk/Tuple.hpp>


namespace wmtk {
class Mesh;
}

namespace wmtk::simplex::utils{


std::vector<Tuple>
unique_homogeneous_simplices(const Mesh& m, PrimitiveType pt, const std::vector<Tuple>& tups);
void unique_homogeneous_simplices_inline(const Mesh& m, PrimitiveType pt, std::vector<Tuple>& tups);
} // namespace wmtk::simplex::internal
