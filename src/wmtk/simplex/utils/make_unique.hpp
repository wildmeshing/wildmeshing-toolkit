#pragma once
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>


namespace wmtk::simplex::utils {
std::vector<Simplex> make_unique(const Mesh& m, const std::vector<Simplex>&);
std::vector<Tuple>
make_unique_tuples(const Mesh& m, const std::vector<Tuple>&, PrimitiveType primitive);
} // namespace wmtk::simplex::utils
