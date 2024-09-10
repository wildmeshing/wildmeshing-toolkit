#include "CDT.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTag.hpp>

#include "internal/CDT.hpp"
#include "internal/CDTOptions.hpp"

namespace wmtk {
namespace components {

using namespace internal;

void CDT(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    CDTOptions options = j.get<CDTOptions>();

    auto trimesh_in = cache.read_mesh(options.input);
    TriMesh& trimesh = static_cast<TriMesh&>(*trimesh_in);

    std::vector<std::array<bool, 4>> local_f_on_input;

    wmtk::logger().info("start CDT ...");

    std::shared_ptr<TetMesh> tm = CDT_internal(trimesh, local_f_on_input, options.inner_only);

    wmtk::logger().info("finished CDT");

    auto surface_handle = tm->register_attribute<int64_t>("surface", PrimitiveType::Triangle, 1);
    auto surface_accessor = tm->create_accessor<int64_t>(surface_handle);

    const auto& tets = tm->get_all(PrimitiveType::Tetrahedron);

    for (int64_t i = 0; i < tets.size(); ++i) {
        const auto& t = tets[i];
        std::array<Tuple, 4> fs = {
            {tm->Mesh::switch_tuples(t, {PV, PE, PF}),
             tm->Mesh::switch_tuples(t, {PE, PF}),
             t,
             tm->Mesh::switch_tuples(t, {PF})}};

        for (int64_t k = 0; k < 4; ++k) {
            if (local_f_on_input[i][k]) {
                surface_accessor.scalar_attribute(fs[k]) = 1;
            } else {
                surface_accessor.scalar_attribute(fs[k]) = 0;
            }
        }
    }

    std::shared_ptr<Mesh> surface_mesh;
    internal::MultiMeshFromTag mmft(*tm, surface_handle, 1);
    mmft.compute_substructure_mesh();
    surface_mesh = tm->get_child_meshes().back();
    mmft.remove_soup();

    wmtk::logger().info("registered surface child mesh to tetmesh {}", options.output);

    std::map<std::string, std::vector<int64_t>> names;

    names["tetmesh"] = tm->absolute_multi_mesh_id();
    names["surface_mesh"] = tm->absolute_multi_mesh_id();

    cache.write_mesh(*tm, options.output, names);
}

} // namespace components
} // namespace wmtk