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

#include <wmtk/components/multimesh/utils/AttributeDescription.hpp>
#include <wmtk/components/multimesh/utils/get_attribute.hpp>
#include <wmtk/utils/cast_attribute.hpp>
#include "fuse.hpp"

wmtk::components::multimesh::NamedMultiMesh& run(

    Params& params)
{
    auto all_meshes = params.collection.all_roots();


    // auto mptr = fuse_eigen(params.collection, params.position_attribute_name);
    //   wmtk::components::tetwild_simplification(*mptr, std::string(position_attribute_name),
    //   1e-3);

    auto [mptr, pairs] = fuse(params.collection, params.alignments, params.position_attribute_name);


    /*
    spdlog::info("Creating tag attributes");
    for (wmtk::PrimitiveType pt : {wmtk::PrimitiveType::Triangle, wmtk::PrimitiveType::Edge}) {
        auto handle = mptr->register_attribute<int64_t>(
            std::string(fmt::format(fmt::runtime(params.tag_format), 0)),
            pt,
            1);
        auto acc = mptr->create_accessor<int64_t, 1>(handle);
        for (const auto& m : mptr->get_all_child_meshes()) {
            if (m == mptr) {
                continue;
            }
            if (m->top_simplex_type() != pt) {
                continue;
            }
            auto id = m->absolute_multi_mesh_id();
            assert(id.size() == 1);
            int64_t ident = id[0];
            for (const wmtk::Tuple& t : m->get_all(pt)) {
                auto s = m->map_to_root(simplex::Simplex(pt, t));
                acc.scalar_attribute(s) = ident;
            }
        }
    }
    */
    std::vector<std::string> names(pairs.size());
    auto pos_handle = wmtk::components::multimesh::utils::get_attribute(
        *mptr,
        wmtk::components::multimesh::utils::AttributeDescription{
            params.position_attribute_name,
            0,
            {}});
    for (auto& [mesh, name] : pairs) {
        wmtk::utils::cast_attribute<double>(
            pos_handle,
            *mesh,
            std::string(params.position_attribute_name));
        auto id = mesh->absolute_multi_mesh_id();
        assert(id.size() == 1);
        names[id[0]] = name;
    }
    nlohmann::ordered_json js;
    js[params.output_name] = names;
    return params.collection.emplace_mesh(*mptr, js);
}
