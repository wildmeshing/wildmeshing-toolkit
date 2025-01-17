#include "fuse.hpp"
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <igl/writeOBJ.h>
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

    std::map<std::string, int64_t> name_to_patch_ids;
    std::map<int64_t, std::string> patch_ids_to_name;

    // corner vertex id -> {mesh_index, vertex index}
    std::map<int64_t, std::set<std::pair<int64_t, wmtk::Tuple>>> corner_vertices;
    for (const auto& [name, mesh] : all_meshes) {
        const auto& em_a = ranges.at(name);
        auto patch_handle =
            mesh.get_attribute_handle_typed<int64_t>("patch_id", wmtk::PrimitiveType::Triangle);
        auto patch_acc = mesh.create_const_accessor(patch_handle);
        auto tups = mesh.get_all(wmtk::PrimitiveType::Triangle);

        int64_t patch_id = patch_acc.const_scalar_attribute(tups[0]);
#if !defined(NDEBUG)
        for (const Tuple& t : tups) {
            int64_t v = patch_acc.const_scalar_attribute(t);
            assert(patch_id == v);
        }
#endif
        name_to_patch_ids[name] = patch_id;
        patch_ids_to_name[patch_id] = name;

        {
            auto corner_vertex_handle =
                mesh.get_attribute_handle_typed<int64_t>("corner_id", wmtk::PrimitiveType::Vertex);
            auto corner_vertex_acc = mesh.create_const_accessor(corner_vertex_handle);
            auto tups = mesh.get_all(wmtk::PrimitiveType::Vertex);

            for (const Tuple& t : tups) {
                int64_t value = corner_vertex_acc.const_scalar_attribute(t);
                if (value != -1) {
                    corner_vertices[value].emplace(patch_id, t);
                }
            }
        }
    }

    Eigen::MatrixX<double> V(total_V, 3);
    Eigen::MatrixX<int64_t> F(total_F, 3);
    VectorX<int64_t> patch_labels(total_F);
    F.setConstant(-1);
    V.setConstant(-1);
    for (auto& [name, em] : ranges) {
        em.F.assign(F, em.V.start());
        em.V.assign(V);
        patch_labels.segment(em.F.start(), em.F.M.rows()).setConstant(name_to_patch_ids.at(name));
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
        }
    }
    std::map<int64_t, std::tuple<int64_t, std::vector<Tuple>>> edge_meshes;

    bool print = true;
    std::vector<std::string> fuse_names_ordered;


    for (const auto& [inds, pairs] : to_fuse) {
        const auto& [ind_a, ind_b] = inds;
        const auto& em_a = ranges.at(get_mesh_name(ind_a));
        const auto& wmtk_mesh = all_meshes.at(get_mesh_name(ind_a));
        std::vector<int64_t> indices;
        std::transform(
            pairs.begin(),
            pairs.end(),
            std::back_inserter(indices),
            [](const auto& pr) -> int64_t { return pr[0]; });
        std::string trim_mesh_name = fmt::format("trim_{}_{}", ind_a, ind_b);
        auto tups = boundary_edges_to_tuples(em_a, indices, true);
        int64_t edge_id = -1;
        {
            auto edge_handle = wmtk_mesh.get_attribute_handle_typed<int64_t>(
                "feature_edge_id",
                wmtk::PrimitiveType::Edge);
            auto edge_acc = wmtk_mesh.create_const_accessor(edge_handle);

            auto edge_tups = boundary_edges_to_tuples(em_a, indices, false);

            edge_id = edge_acc.const_scalar_attribute(edge_tups[0]);
#if !defined(NDEBUG)
            for (const Tuple& t : edge_tups) {
                assert(edge_id == edge_acc.const_scalar_attribute(t));
            }
            assert(edge_id != -1);
#endif
        }
        edge_meshes[edge_id] = std::make_tuple(ind_a, std::move(tups));
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
            auto eb = EB.row(k);
            eb << current_vertex_size + k, current_vertex_size + k + 1;

            size_t global_edge_index = current_edge_size + k;
            auto& m = map[global_edge_index];

            m[0] = Tuple(0, -1, -1, global_edge_index);
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
        int64_t max_index = -1;
        for (const auto& [id, _] : corner_vertices) {
            assert(id >= 0);
            max_index = std::max<int64_t>(id, max_index);
        }
        point_tuples.resize(max_index + 1);
        assert(point_tuples.size() == corner_vertices.size());

        for (const auto& [id, pairs] : corner_vertices) {
            for (const auto& [mesh_id, t] : pairs) {
                const auto& em = ranges.at(patch_ids_to_name.at(mesh_id));
                const Tuple t2 = em.update_to_fused(t);
                //const Tuple t2 = t; // em.update_to_fused(t);
                assert(mptr->is_valid(t2));
                point_tuples[id] = std::array<Tuple, 2>{{Tuple(-1, -1, -1, id), t2}};
                break;
            }
        }
        auto pm = std::make_shared<wmtk::PointMesh>();
        pm->initialize(point_tuples.size());
        for (const auto& [a, b] : point_tuples) {
            assert(!a.is_null());
            assert(!pm->is_removed(a));
            assert(pm->is_valid(a));
            assert(mptr->is_valid(b));
        }
        mptr->register_child_mesh(pm, point_tuples);
        named_meshes.emplace_back(pm, "critical_points");
    }

    return {mptr, named_meshes};
}
