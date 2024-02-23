#include "multimesh_from_tag.hpp"

#include <deque>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>

#include "internal/MultiMeshFromTag.hpp"
#include "internal/MultiMeshFromTagOptions.hpp"

namespace wmtk {
namespace components {

using namespace internal;

void multimesh_from_tag(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    MultiMeshFromTagOptions options = j.get<MultiMeshFromTagOptions>();

    auto mesh_in = cache.read_mesh(options.input);

    Mesh& mesh = static_cast<Mesh&>(*mesh_in);

    attribute::MeshAttributeHandle substructure_label =
        base::get_attribute(mesh, options.substructure_label);

    const int64_t substructure_value = options.substructure_value;

    MultiMeshFromTag mmft(mesh, substructure_label, substructure_value);
    mmft.compute_substructure_mesh();

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps =
            base::get_attributes(cache, mesh, options.pass_through);
        keeps.emplace_back(substructure_label);
        mesh.clear_attributes(keeps);
    }

    {
        std::map<std::string, std::vector<int64_t>> names;
        names[options.input] = mesh.absolute_multi_mesh_id();
        names[options.output] = mmft.substructure()->absolute_multi_mesh_id();
        names[options.output + "_soup"] =
            mmft.substructure_soup()->absolute_multi_mesh_id(); // TODOfix: should be deleted
        cache.write_mesh(mesh, options.output, names); // TODOfix: not sure if that is sufficient
    }
}
} // namespace components
} // namespace wmtk
