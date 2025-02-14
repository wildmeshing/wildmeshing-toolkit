#include "wildmeshing.hpp"

#include "internal/wildmeshing2d.hpp"
#include "internal/wildmeshing3d.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components {

using namespace internal;

std::vector<std::pair<std::shared_ptr<Mesh>, std::string>> wildmeshing(
    const WildMeshingOptions& option)
{
    if (option.input_mesh->top_simplex_type() == PrimitiveType::Triangle) {
        return wildmeshing2d(option);
    } else {
        return wildmeshing3d(option);
    }

    assert(false);
    return {};
}

void wildmeshing(const WildMeshingEmbeddingOptions& options) {
    log_and_throw_error("not implemented");
}

} // namespace wmtk::components
