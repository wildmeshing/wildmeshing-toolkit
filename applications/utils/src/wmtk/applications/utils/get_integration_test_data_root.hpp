#pragma once
#include <filesystem>

namespace wmtk::applications::utils {
    // allows for some applications to retrieve the data folder used for their integration tests using the json manifest that exists in each build folder (typically as "test_config.json"). Requires knowing which app they are (i.e isotropic_remeshing_app) as app_path
    std::filesystem::path get_integration_test_data_root(const std::filesystem::path& js_path, const std::filesystem::path& app_path) ;
}
