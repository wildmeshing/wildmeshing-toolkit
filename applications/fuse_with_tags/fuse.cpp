#include "fuse.hpp"
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <igl/writeOBJ.h>
#include <spdlog/spdlog.h>
#include <set>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/components/multimesh/from_facet_bijection.hpp>
#include <wmtk/components/multimesh/from_facet_surjection.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/io/internal/get_attribute_handles.hpp>
#include <wmtk/utils/DisjointSet.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "edges_to_tuples.hpp"

#include <wmtk/components/multimesh/MeshCollection.hpp>

#include <wmtk/TriMesh.hpp>
#include "utils.hpp"
namespace wmtk::components::multimesh {
class MeshCollection;
}

std::pair<
    std::shared_ptr<wmtk::TriMesh>,
    std::vector<std::tuple<std::shared_ptr<Mesh>, std::string>>>
fuse(
    wmtk::components::multimesh::MeshCollection& mc,
    const std::map<std::array<int64_t, 2>, std::vector<std::array<int64_t, 2>>>& to_fuse,
    // const std::map<std::array<int64_t, 2>, std::vector<std::array<Tuple, 2>>>& to_fuse,
    const std::string_view& position_attribute_name)
{
    std::vector<std::tuple<std::shared_ptr<Mesh>, std::string>> named_meshes;
    auto all_meshes = mc.all_roots();
    auto ranges = get_meshes(mc, position_attribute_name);
    auto get_mesh_name = [&](int64_t index) -> std::string { return fmt::format("{}", index); };
    int total_F = 0;
    int total_V = 0;
    for (const auto& [name, em] : ranges) {
        total_V = std::max<int>(total_V, em.V.end());
        total_F = std::max<int>(total_F, em.F.end());
    }

    Eigen::MatrixX<double> V(total_V, 3);
    Eigen::MatrixX<int64_t> F(total_F, 3);
    VectorX<int64_t> patch_labels(total_F);
    F.setConstant(-1);
    V.setConstant(-1);
    int64_t index = 0; // TODO: replace
    for (auto& [name, em] : ranges) {
        em.F.assign(F, em.V.start());
        em.V.assign(V);
        patch_labels.segment(em.F.start(), em.F.M.rows()).setConstant(index++);
    }


    auto patch_mesh = std::make_shared<wmtk::TriMesh>();
    named_meshes.emplace_back(patch_mesh, "patches");

    patch_mesh->initialize(F);
    wmtk::mesh_utils::set_matrix_attribute(
        patch_labels,
        "patch_labels",
        wmtk::PrimitiveType::Triangle,
        *patch_mesh);
    wmtk::utils::DisjointSet Vsets(total_V);


    {
        {
            std::vector<size_t> roots = Vsets.roots();
            std::map<int64_t, int64_t> root_indices;
            for (size_t j = 0; j < roots.size(); ++j) {
                root_indices[roots[j]] = j;
            }
            std::vector<size_t> counts(roots.size());
            for (int j = 0; j < total_V; ++j) {
                counts[root_indices.at(Vsets.get_root(j))]++;
            }
            // spdlog::info("{}", fmt::join(counts, ","));
        }
    }
    std::map<int64_t, std::tuple<int64_t, std::vector<Tuple>>> edge_meshes;

    bool print = true;
    std::vector<std::string> fuse_names_ordered;

    int64_t edge_mesh_index = 0; // TODO replace when we have numbers for the pairs

    for (const auto& [inds, pairs] : to_fuse) {
        const auto& [ind_a, ind_b] = inds;
        const auto& em_a = ranges.at(get_mesh_name(ind_a));
        std::vector<int64_t> indices;
        std::transform(
            pairs.begin(),
            pairs.end(),
            std::back_inserter(indices),
            [](const auto& pr) -> int64_t { return pr[0]; });
        std::string trim_mesh_name = fmt::format("trim_{}_{}", ind_a, ind_b);
        spdlog::info(
            "Trim {} has index {}, with {} pairs {}",
            trim_mesh_name,
            edge_mesh_index,
            pairs.size(),
            fmt::join(indices, ","));
        auto tups = boundary_edges_to_tuples(em_a, indices);
        edge_meshes[edge_mesh_index++] = std::make_tuple(ind_a, std::move(tups));
    }

    for (const auto& [inds, pairs] : to_fuse) {
        const auto& [ind_a, ind_b] = inds;
        const auto& em_a = ranges.at(get_mesh_name(ind_a));
        const auto& em_b = ranges.at(get_mesh_name(ind_b));


        for (auto [ta, tb] : pairs) {
            int64_t ota = ta;
            int64_t otb = tb;
            ta += em_a.V.start();
            tb += em_b.V.start();
            Vsets.merge(ta, tb);
        }
    }
    {
        std::vector<size_t> roots = Vsets.roots();
        std::map<int64_t, int64_t> root_indices;
        for (size_t j = 0; j < roots.size(); ++j) {
            root_indices[roots[j]] = j;
        }

        for (int j = 0; j < F.size(); ++j) {
            int64_t& v = F(j);
            v = root_indices.at(Vsets.get_root(v));
        }

        for (const auto& v : roots) {
            assert(v < V.rows());
        }
        V = V(roots, Eigen::all).eval();
    }

    auto mptr = std::make_shared<wmtk::TriMesh>();

    mptr->initialize(F);
    auto pos_handle = wmtk::mesh_utils::set_matrix_attribute(
        V,
        std::string(position_attribute_name),
        wmtk::PrimitiveType::Vertex,
        *mptr);

    auto pos_acc = mptr->create_accessor<double>(pos_handle);

    assert(mptr->is_connectivity_valid());
    /*
    for (const auto& [name, em] : ranges) {
        // spdlog::info("Registering {} as child mesh", name);
        auto& m = const_cast<wmtk::Mesh&>(all_meshes.at(name));

        named_meshes.emplace_back(m.shared_from_this(), name);
        std::vector<int64_t> i(em.F.M.rows());
        assert(em.F.M.rows() == m.get_all(wmtk::PrimitiveType::Triangle).size());
        std::iota(i.begin(), i.end(), em.F.start());

        wmtk::components::multimesh::from_facet_surjection(*mptr, m, i);
    }

    for (const auto& [name, pr] : edge_meshes) {
        const auto& [mesh_id, tups] = pr;
        const auto& mesh_a = all_meshes.at(get_mesh_name(mesh_id));
        std::vector<std::array<Tuple, 2>> map(tups.size());

        // spdlog::info("Making edgemesh {} form mesh id {}", name, mesh_id);

        for (size_t j = 0; j < tups.size(); ++j) {
            mesh_a.is_valid(tups[j]);
            map[j][0] = Tuple(0, -1, -1, j);
            map[j][1] = mesh_a.map_to_parent_tuple(simplex::Simplex(PrimitiveType::Edge, tups[j]));
        }
        RowVectors2l E(tups.size(), 2);
        for (int k = 0; k < tups.size(); ++k) {
            E.row(k) << k, k + 1;
        }
        auto em_ptr = std::make_shared<wmtk::EdgeMesh>();
        em_ptr->initialize(E);

        mptr->register_child_mesh(em_ptr, map);
        named_meshes.emplace_back(em_ptr, name);
    }
    */
    wmtk::components::multimesh::from_facet_bijection(*mptr, *patch_mesh);

    size_t total_edges = 0;
    for (const auto& [name, pr] : edge_meshes) {
        const auto& [mesh_id, tups] = pr;
        total_edges += tups.size();
    }
    RowVectors2l E(total_edges, 2);
    E.setConstant(-1);
    VectorX<int64_t> edge_labels(total_edges);

    size_t current_vertex_size = 0;
    size_t current_edge_size = 0;
    std::vector<std::array<Tuple, 2>> map(total_edges);
    for (const auto& [trim_index, pr] : edge_meshes) {
        const auto& [mesh_id, tups] = pr;
        const auto& mesh_a = all_meshes.at(get_mesh_name(mesh_id));
        const auto& range_a = ranges.at(get_mesh_name(mesh_id));

        edge_labels.segment(current_edge_size, tups.size()).setConstant(trim_index);

        auto EB = E.block(current_edge_size, 0, tups.size(), 2);
        for (int k = 0; k < tups.size(); ++k) {
            EB.row(k) << current_vertex_size + k, current_vertex_size + k + 1;
            size_t global_edge_index = current_edge_size + k;
            auto& m = map[global_edge_index];

            m[0] = Tuple(1, -1, -1, global_edge_index);
            m[1] = patch_mesh->map_to_parent_tuple(simplex::Simplex(PrimitiveType::Edge, tups[k]));
            assert(m[1] == tups[k]);
        }
        current_vertex_size += tups.size() + 1;
        current_edge_size += tups.size();
    }
    auto em_ptr = std::make_shared<wmtk::EdgeMesh>();
    em_ptr->initialize(E);

    RowVectors3l FE(total_edges, 3);

    FE.leftCols<2>() = E;
    FE.col(2) = edge_labels;


    wmtk::mesh_utils::set_matrix_attribute(
        edge_labels,
        "edge_labels",
        wmtk::PrimitiveType::Edge,
        *em_ptr);
    mptr->register_child_mesh(em_ptr, map);
    named_meshes.emplace_back(em_ptr, "trims");

    {
        std::vector<std::array<Tuple, 2>> point_tuples;
        for (const auto& t : em_ptr->get_all(wmtk::PrimitiveType::Vertex)) {
            simplex::Simplex s(wmtk::PrimitiveType::Vertex, t);
            auto ss = em_ptr->map(*em_ptr, s);
            if (ss.size() > 1) {
                point_tuples.emplace_back(std::array<Tuple, 2>{
                    {Tuple(-1, -1, -1, point_tuples.size()), em_ptr->map_to_root(s).tuple()}});
            }
        }
        auto pm = std::make_shared<wmtk::PointMesh>();
        pm->initialize(point_tuples.size());
        mptr->register_child_mesh(pm, point_tuples);
        named_meshes.emplace_back(pm, "critical_points");
    }

    return {mptr, named_meshes};
}
