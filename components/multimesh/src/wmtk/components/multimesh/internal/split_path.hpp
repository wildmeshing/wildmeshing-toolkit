#pragma once
#include <ranges>
#include <string_view>
#include <sstream>

namespace wmtk::components::multimesh::internal {

inline auto split_path(const std::string_view& view)
{
#if defined(WMTK_ENABLED_CPP20)
    using namespace std;
    return std::ranges::views::split(view, "."sv) |
           std::views::transform([](const auto& r) noexcept -> std::string_view {
               return std::string_view(r.begin(), r.end());
           });

#else
    std::vector<std::string> tokens;
    if(view.empty()) {
        tokens.emplace_back("");
    
    } else {
    std::string v = std::string(view);
    std::istringstream iss(v);
    std::string token;
    if(v.size() > 0 && v[0] == '.') {
        tokens.emplace_back("");
    }
    while (std::getline(iss, token, '.')) {
        if (!token.empty())
            tokens.push_back(token);
    }
    }
    return tokens;
#endif
}
} // namespace wmtk::components::multimesh::internal
