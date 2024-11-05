#include "import_cache.hpp"

#include <wmtk/components/utils/resolve_path.hpp>
#include <wmtk/utils/Logger.hpp>

#include "ImportCacheOptions.hpp"

namespace wmtk::components {

void import_cache(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    ImportCacheOptions options = j.get<ImportCacheOptions>();

    std::string import_location =
        wmtk::utils::resolve_path(options.folder.string(), paths.root_path);

    if (!std::filesystem::exists(import_location)) {
        log_and_throw_error("Cannot import cache, folder {} does not exist", import_location);
    }

    if (!cache.import_cache(import_location)) {
        log_and_throw_error("Could not import cache from {}", import_location);
    }
}
} // namespace wmtk::components