#include "json_utils.hpp"

namespace nlohmann {

void adl_serializer<std::filesystem::path>::to_json(json& j, const std::filesystem::path& p)
{
    j = p.string();
}

void adl_serializer<std::filesystem::path>::from_json(const json& j, std::filesystem::path& p)
{
    p = j.get<std::string>();
}

} // namespace nlohmann
