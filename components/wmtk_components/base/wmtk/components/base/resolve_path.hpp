#pragma once

#include <string>

namespace wmtk::components::base {

std::string resolve_path(
    const std::string& path,
    const std::string& input_file_path,
    const bool only_if_exists = false);

}
