#include "marching_window.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/components/base/resolve_path.hpp>

#include "internal/MarchingWindow.hpp"
#include "internal/MarchingWindowOptions.hpp"

namespace wmtk::components {

void marching_window(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;
}

} // namespace wmtk::components
