#pragma once
#include <filesystem>

namespace wmtk::applications::utils {
    std::filesystem::path get_integration_test_data_root(const std::filesystem::path& js_path, const std::filesystem::path& app_path) ;
}
