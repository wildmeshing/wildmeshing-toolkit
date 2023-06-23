#pragma once
#include <map>
#include <nlohmann/json.hpp>

#include "internal/InputOptions.h"

namespace wmtk {
namespace components {
inline void input(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    InputOptions options = j.get<InputOptions>();

    files[options.name] = options.file;
}
} // namespace components
} // namespace wmtk