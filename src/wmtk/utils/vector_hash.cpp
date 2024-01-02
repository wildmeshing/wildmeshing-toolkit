#include "vector_hash.hpp"
#include <algorithm>
#include <functional>
#include <string>
#include <string_view>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::utils {
namespace {
template <typename T>
std::size_t _vector_hash_impl(const std::vector<T>& data)
{
    // wraps the passed data as the contents of a std::string_view to take
    // advantage of std::string_view's hash function. NOTE that
    // std::string_view's hash function is probably not consistent between
    // platforms. In fact, on libstdcxx depends on <bits/hash_bytes.h> which
    // states that the STL _Hash_bytes is not a constant API per release.
    // as such TODO: find a platform-agnostic hash function
    using SVType = std::string_view::value_type;
    static_assert(sizeof(T) % sizeof(SVType) == 0);
    constexpr static size_t size_ratio = sizeof(T) / sizeof(SVType);

    std::string_view view(reinterpret_cast<const SVType*>(data.data()), size_ratio * data.size());
    return std::hash<std::string_view>{}(view);
}
} // namespace

std::size_t vector_hash(const std::vector<size_t>& data)
{
    return _vector_hash_impl(data);
}
std::size_t vector_hash(const std::vector<int64_t>& data)
{
    return _vector_hash_impl(data);
}
std::size_t vector_hash(const std::vector<double>& data)
{
    return _vector_hash_impl(data);
}
std::size_t vector_hash(const std::vector<char>& data)
{
    return _vector_hash_impl(data);
}
std::size_t vector_hash(const std::vector<Rational>& data)
{
    std::vector<size_t> hashes;
    std::transform(data.begin(), data.end(), std::back_inserter(hashes), [](const Rational& r) {
        std::vector<size_t> v;
        std::hash<std::string> h;
        v.emplace_back(h(r.numerator()));
        v.emplace_back(h(r.denominator()));
        return vector_hash(v);
    });
    return vector_hash(hashes);
}
} // namespace wmtk::utils
