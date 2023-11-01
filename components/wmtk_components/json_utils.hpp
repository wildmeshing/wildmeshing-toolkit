#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>

namespace nlohmann {
template <>
// json serialization of path
struct adl_serializer<std::filesystem::path>
{
    static void to_json(json& j, const std::filesystem::path& p) { j = p.string(); }

    static void from_json(const json& j, std::filesystem::path& p) { p = j.get<std::string>(); }
};
} // namespace nlohmann
