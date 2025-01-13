#include <fmt/format.h>
#include <igl/writeOBJ.h>
#include <spdlog/spdlog.h>
#include <numeric>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/components/tetwild_simplification/tetwild_simplification.hpp>

#include <igl/remove_duplicate_vertices.h>
#include <set>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/components/multimesh/from_facet_surjection.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include "run.hpp"


namespace {

template <typename T>
struct EigenMesh
{
    Eigen::Index start() const { return m_start; }
    Eigen::Index end() const { return m_start + M.rows(); }

    Eigen::Index m_start = 0;
    Eigen::MatrixX<T> M;

    template <typename D>
    void assign(Eigen::MatrixBase<D>& m) const
    {
        m.block(start(), 0, M.rows(), M.cols()) = M;
        // m(Eigen::seq(start(), end()), Eigen::all) = M;
    }
};
struct EigenMeshes
{
    EigenMesh<double> V;
    EigenMesh<int64_t> F;
};
} // namespace

wmtk::components::multimesh::NamedMultiMesh& run(
    wmtk::components::multimesh::MeshCollection& collection,
    const std::string_view& output_name,
    const std::string_view& tag_format,
    const std::string_view& position_attribute_name)
{
    auto all_meshes = collection.all_roots();

    std::map<std::string, EigenMeshes> ranges;


    wmtk::utils::EigenMatrixWriter writer;
    writer.set_position_attribute_name(position_attribute_name);

    int total_V = 0;
    int total_F = 0;
    {
        for (const auto& [name, mesh] : all_meshes) {
            assert(mesh.is_connectivity_valid());
            spdlog::info("sizing {}", name);
            mesh.serialize(writer);

            EigenMeshes em;
            em.V.m_start = total_V;
            em.F.m_start = total_F;


            writer.get_position_matrix(em.V.M);
            writer.get_FV_matrix(em.F.M);

            total_V = em.V.end();
            total_F = em.F.end();
            ranges[name] = std::move(em);
        }
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
    wmtk::mesh_utils::set_matrix_attribute(SV,std::string(position_attribute_name), wmtk::PrimitiveType::Vertex, *mptr);
    wmtk::components::tetwild_simplification(*mptr, std::string(position_attribute_name), 1e-3);
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

    return collection.emplace_mesh(*mptr, output_name);
}
