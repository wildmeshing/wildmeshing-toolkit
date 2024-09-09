#include "CDT.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/components/multimesh_from_tag/multimesh_from_tag.hpp>

#include "internal/CDT.hpp"
#include "internal/CDTOptions.hpp"

namespace wmtk {
namespace components {

using namespace internal;

void CDT(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    CDTOptions options = j.get<CDTOptions>();

    auto trimesh_in = cache.read_mesh(options.input);
    TriMesh& trimesh = static_cast<TriMesh&>(*trimesh_in);

    std::vector<std::array<bool, 4>> local_f_on_input;

    std::shared_ptr<TetMesh> tm = CDT_internal(trimesh, local_f_on_input);
}

} // namespace components
} // namespace wmtk