#include "export_cache.hpp"

#include <wmtk/components/utils/resolve_path.hpp>
#include <wmtk/utils/Logger.hpp>

#include "ExportCacheOptions.hpp"

namespace wmtk::components {

void export_cache(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    ExportCacheOptions options = j.get<ExportCacheOptions>();

    std::string export_location =
        wmtk::utils::resolve_path(options.folder.string(), paths.root_path);

    if (std::filesystem::exists(export_location)) {
        log_and_throw_error("Cannot export cache, folder {} already exists", export_location);
    }

    if (!cache.export_cache(export_location)) {
        log_and_throw_error("Could not export cache from {}", export_location);
    }
}
} // namespace wmtk::components