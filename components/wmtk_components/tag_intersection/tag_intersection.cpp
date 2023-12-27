#include "tag_intersection.hpp"

#include <deque>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>

#include "internal/TagIntersection.hpp"
#include "internal/TagIntersectionOptions.hpp"

namespace wmtk {
namespace components {
void tag_intersection(const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    TagIntersectionOptions options = j.get<TagIntersectionOptions>();

    auto mesh_in = cache.read_mesh(options.input);


    std::vector<std::tuple<MeshAttributeHandle<long>, long>> input_tags;
    for (const auto& [name, ptype, val] : options.input_tags) {
        assert(ptype != PrimitiveType::Tetrahedron);

        auto handle = mesh_in->get_attribute_handle<long>(name, ptype);
        input_tags.emplace_back(std::make_tuple(handle, val));
    }

    std::vector<std::tuple<MeshAttributeHandle<long>, long>> output_tags;
    for (const auto& [name, ptype, val] : options.output_tags) {
        auto handle = mesh_in->register_attribute<long>(name, ptype, 1);
        output_tags.emplace_back(std::make_tuple(handle, val));
    }

    switch (mesh_in->top_simplex_type()) {
    case PrimitiveType::Face: {
        TriMesh& m = static_cast<TriMesh&>(*mesh_in);
        wmtk::components::TagIntersection tag_intersection;
        tag_intersection.compute_intersection(m, input_tags, output_tags);
        break;
    }
    case PrimitiveType::Tetrahedron: {
        TetMesh& m = static_cast<TetMesh&>(*mesh_in);
        wmtk::components::TagIntersection tag_intersection;
        tag_intersection.compute_intersection(m, input_tags, output_tags);
        break;
    }
    default: {
        log_and_throw_error(
            "Works only for faces and tetrahedrons, error-type: {}",
            mesh_in->top_simplex_type());
        break;
    }
    }

    cache.write_mesh(*mesh_in, options.output);
}
} // namespace components
} // namespace wmtk
