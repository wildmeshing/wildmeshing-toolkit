#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <numeric>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/tetwild_simplification/tetwild_simplification.hpp>
#include "fuse_eigen.hpp"

#include <set>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include "run.hpp"

#include "fuse.hpp"

wmtk::components::multimesh::NamedMultiMesh& run(

    Params& params)
{
    auto all_meshes = params.collection.all_roots();


    // auto mptr = fuse_eigen(params.collection, params.position_attribute_name);
    //   wmtk::components::tetwild_simplification(*mptr, std::string(position_attribute_name),
    //   1e-3);

    auto mptr = fuse(params.collection, params.alignments, params.position_attribute_name);


    spdlog::info("Creating tag attributes");
    for (wmtk::PrimitiveType pt : {wmtk::PrimitiveType::Vertex, wmtk::PrimitiveType::Edge}) {
        auto handle = mptr->register_attribute<int64_t>(
            std::string(fmt::format(fmt::runtime(params.tag_format), 0)),
            pt,
            1);
        auto acc = mptr->create_accessor<int64_t, 1>(handle);
        spdlog::info("Going into simplices");
        int count = 0;
        for (const wmtk::Tuple& t : mptr->get_all(pt)) {
            if (mptr->mappable_child_meshes(wmtk::simplex::Simplex(pt, t)).size() > 1) {
                acc.scalar_attribute(t) = 1;
            }
        }
    }
    return params.collection.emplace_mesh(*mptr, params.output_name);
}
