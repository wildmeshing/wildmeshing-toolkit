#pragma once
#include <cstdint>
#include <vector>

namespace wmtk {
class Rational;
}

namespace wmtk::utils {
// computes a hash of the input data. Useful for implementing merkle trees
std::size_t vector_hash(const std::vector<std::size_t>& data);
std::size_t vector_hash(const std::vector<int64_t>& data);
std::size_t vector_hash(const std::vector<double>& data);
std::size_t vector_hash(const std::vector<char>& data);
std::size_t vector_hash(const std::vector<Rational>& data);
} // namespace wmtk::utils
