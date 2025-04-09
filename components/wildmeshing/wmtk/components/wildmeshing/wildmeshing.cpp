#include "wildmeshing.hpp"

#include "internal/wildmeshing2d.hpp"
#include "internal/wildmeshing3d.hpp"
#include "internal/wildmeshing_embedding_2d.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components {

using namespace internal;

std::vector<std::pair<std::shared_ptr<Mesh>, std::string>> wildmeshing(
    const WildMeshingOptions& options)
{
    if (options.use_embedding) {
        if (options.input_mesh->top_simplex_type() == PrimitiveType::Triangle) {
            return wildmeshing_embedding_2d(options);
        } else {
            log_and_throw_error("3D embedding not implemented yet.");
            // return wildmeshing3d(option);
        }
    }

    if (options.input_mesh->top_simplex_type() == PrimitiveType::Triangle) {
        return wildmeshing2d(options);
    } else {
        return wildmeshing3d(options);
    }

    assert(false);
    return {};
}

} // namespace wmtk::components
