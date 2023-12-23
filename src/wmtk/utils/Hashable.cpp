#include "Hashable.hpp"
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <stdexcept>
#include <vector>


namespace wmtk::utils {
Hashable::Hashable() = default;
Hashable::Hashable(const Hashable&) = default;
Hashable::Hashable(Hashable&&) = default;
Hashable& Hashable::operator=(const Hashable&) = default;
Hashable& Hashable::operator=(Hashable&&) = default;
Hashable::~Hashable() = default;
std::size_t Hashable::hash() const
{
    auto ch = child_hashes();
    // if (ch.size() == 0) {
    //     throw std::runtime_error(
    //         "Hash base implementation was called without any child hashes. Either guarantee there
    //         " "is a child in child_hashes overload or overload this hash() function directly");
    // }
    std::vector<std::string> data;
    for (const auto& [key, value] : ch) {
        data.emplace_back(fmt::format("({}:{})", key, value));
    }

    std::string strdata = fmt::format("{}", fmt::join(data, ","));
    return std::hash<std::string>{}(strdata);
}

std::map<std::string, size_t> Hashable::child_hashes() const
{
    return {};
}
} // namespace wmtk::utils
