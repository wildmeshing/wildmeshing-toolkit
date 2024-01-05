#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>

namespace nlohmann {
// json serialization of path
template <>
struct adl_serializer<std::filesystem::path>
{
    static void to_json(json& j, const std::filesystem::path& p);

    static void from_json(const json& j, std::filesystem::path& p);
};
} // namespace nlohmann
