#pragma once

#include <string>

namespace wmtk::components::utils {

std::string resolve_path(
    const std::string& path,
    const std::string& input_file_path,
    const bool only_if_exists = false);

}
