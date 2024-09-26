#pragma once
#include <ranges>
#include <string_view>

namespace wmtk::components::input::internal {

auto split_path(const std::string_view& view)
{
    using namespace std;
    return std::ranges::views::split(view, "."sv) |
           std::views::transform([](const auto& r) noexcept -> std::string_view {
               return std::string_view(r.begin(), r.end());
           });
}
} // namespace wmtk::components::input::internal
