#include "fuse_eigen.hpp"
#include <igl/remove_duplicate_vertices.h>
#include <igl/writeOBJ.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <numeric>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/components/multimesh/from_facet_surjection.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "utils.hpp"

std::shared_ptr<wmtk::TriMesh> fuse_eigen(
    wmtk::components::multimesh::MeshCollection& mc,
    const std::string_view& position_attribute_name,
    const std::string_view& tag_format)
{
    auto all_meshes = mc.all_roots();
    auto ranges = get_meshes(mc, position_attribute_name);


    int total_V = 0;
    int total_F = 0;
    for (const auto& [name, em] : ranges) {
        total_V = std::max<int>(total_V, em.V.end());
        total_F = std::max<int>(total_F, em.F.end());
    }

    spdlog::info("Creating matrices V size {}, F size {}", total_V, total_F);
    Eigen::MatrixXd V(total_V, 3);
    Eigen::MatrixX<int64_t> F(total_F, 3);

    for (const auto& [name, em] : ranges) {
        em.V.assign(V);
        em.F.assign(F);
    }
    Eigen::MatrixXd SV;
    Eigen::MatrixX<int64_t> SVI, SVJ, SF;
    spdlog::info("Calling remove_Duplicate_vertices");

    igl::remove_duplicate_vertices(V, F, 0, SV, SVI, SVJ, SF);

    spdlog::info("Creating old inddex map");
    // std::map<int64_t, std::set<int64_t>> old_indices;
    // for (int j = 0; j < V.rows(); ++j) {
    //     old_indices[SVI(j)].emplace(j);
    // }


    spdlog::info("Creating trimesh");
    auto mptr = std::make_shared<wmtk::TriMesh>();
    igl::writeOBJ("hi", SV, SF);

    mptr->initialize(SF);
    wmtk::mesh_utils::set_matrix_attribute(
        SV,
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

    return mptr;
}
