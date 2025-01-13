#include "fuse.hpp"
#include <fmt/format.h>
#include <igl/writeOBJ.h>
#include <spdlog/spdlog.h>
#include <wmtk/components/multimesh/from_facet_surjection.hpp>
#include <wmtk/io/internal/get_attribute_handles.hpp>
#include <wmtk/utils/DisjointSet.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <wmtk/components/multimesh/MeshCollection.hpp>

#include <wmtk/TriMesh.hpp>
#include "utils.hpp"
namespace wmtk::components::multimesh {
class MeshCollection;
}

std::shared_ptr<wmtk::TriMesh> fuse(
    wmtk::components::multimesh::MeshCollection& mc,
    const std::map<std::array<int64_t, 2>, std::vector<std::array<int64_t, 2>>>& to_fuse,
    // const std::map<std::array<int64_t, 2>, std::vector<std::array<Tuple, 2>>>& to_fuse,
    const std::string_view& position_attribute_name,
    const std::string& name_format,
    const std::string_view& tag_format)
{
    auto all_meshes = mc.all_roots();
    auto ranges = get_meshes(mc, position_attribute_name);
    auto get_mesh_name = [&](int64_t index) -> std::string {
        return fmt::format(fmt::runtime(name_format), index);
    };
    int total_F = 0;
    int total_V = 0;
    for (const auto& [name, em] : ranges) {
        total_V = std::max<int>(total_V, em.V.end());
        total_F = std::max<int>(total_F, em.F.end());
    }

    Eigen::MatrixX<int64_t> V(total_V, 3);
    Eigen::MatrixX<int64_t> F(total_F, 3);
    for (auto& [name, em] : ranges) {
        em.F.M.array() = em.F.M.array() + em.V.start();
        em.F.assign(F);
        em.V.assign(V);
    }

    wmtk::utils::DisjointSet Vsets(total_V);

    /*
    auto new_tup = [](const EigenMeshes& em, const Tuple& t) {
        auto gid = wmtk::utils::TupleInspector::global_cid(t);
        auto vid = wmtk::utils::TupleInspector::local_vid(t);
        auto eid = wmtk::utils::TupleInspector::local_eid(t);
        auto fid = wmtk::utils::TupleInspector::local_fid(t);
        gid += em.V.start();
        return Tuple(gid, vid, eid, fid);
    };

    for (const auto& [inds, tuples] : to_fuse) {
        const auto& [ind_a, ind_b] = inds;
        const auto& em_a = ranges.at(get_mesh_name(ind_a));
        const auto& em_b = ranges.at(get_mesh_name(ind_b));

        for (const auto& [ta, tb] : tuples) {
            auto nta = new_tup(em_a, ta);
            auto bta = new_tup(em_b, ta);
        }
    }
    */
    for (const auto& [inds, pairs] : to_fuse) {
        const auto& [ind_a, ind_b] = inds;
        const auto& em_a = ranges.at(get_mesh_name(ind_a));
        const auto& em_b = ranges.at(get_mesh_name(ind_b));

        for (auto [ta, tb] : pairs) {
            ta += em_a.V.start();
            tb += em_b.V.start();
            Vsets.merge(ta, tb);
        }
    }
    std::vector<size_t> roots = Vsets.roots();
    std::map<int64_t, int64_t> root_indices;
    for (size_t j = 0; j < roots.size(); ++j) {
        root_indices[roots[j]] = j;
    }
    for (int j = 0; j < F.size(); ++j) {
        int64_t& v = F(j);
        v = root_indices.at(Vsets.get_root(v));
    }

    V = V(roots, Eigen::all);


    spdlog::info("Creating trimesh");
    auto mptr = std::make_shared<wmtk::TriMesh>();
    igl::writeOBJ("hi", V, F);

    mptr->initialize(F);
    wmtk::mesh_utils::set_matrix_attribute(
        V,
        std::string(position_attribute_name),
        wmtk::PrimitiveType::Vertex,
        *mptr);

    assert(mptr->is_connectivity_valid());
    for (const auto& [name, em] : ranges) {
        spdlog::info("Registering {} as child mesh", name);
        auto& m = const_cast<wmtk::Mesh&>(all_meshes.at(name));
        std::vector<int64_t> i(em.F.M.rows());
        assert(em.F.M.rows() == m.get_all(wmtk::PrimitiveType::Triangle).size());
        std::iota(i.begin(), i.end(), em.F.start());

        wmtk::components::multimesh::from_facet_surjection(*mptr, m, i);
    }
    spdlog::info("Creating tag attributes");
    for (wmtk::PrimitiveType pt : {wmtk::PrimitiveType::Vertex, wmtk::PrimitiveType::Edge}) {
        auto handle = mptr->register_attribute<int64_t>(
            std::string(fmt::format(fmt::runtime(tag_format), 0)),
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

    return mptr;
}
