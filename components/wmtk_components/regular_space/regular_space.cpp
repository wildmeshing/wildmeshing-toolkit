#include "regular_space.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/io/Cache.hpp>

#include "internal/RegularSpace.hpp"
#include "internal/RegularSpaceOptions.hpp"

namespace wmtk::components {

void regular_space(const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    RegularSpaceOptions options = j.get<RegularSpaceOptions>();

    // input
    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    Mesh& mesh = static_cast<Mesh&>(*mesh_in);

    RegularSpace rs(mesh);
    rs.regularize_tags(options.tags);

    cache.write_mesh(mesh, options.output);
}

} // namespace wmtk::components
