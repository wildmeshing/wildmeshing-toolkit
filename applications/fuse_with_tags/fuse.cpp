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
    auto check_degen = [](const std::string_view& name, const auto& V, const auto& F) {
        bool ok = true;
        for (int j = 0; j < F.rows(); ++j) {
            auto f = F.row(j);
            for (int k = 0; k < F.cols(); ++k) {
                int64_t fk = f(k);
                for (int l = 0; l < F.cols(); ++l) {
                    if (k == l) {
                        continue;
                    }
                    int64_t fl = f(l);
                    if (fk == fl) {
                        spdlog::info("Fail on mesh {}, face {}:  {}", name, j, fmt::join(f, ","));
                        ok = false;
                    }
                }
            }
        }
        if (!ok) {
            throw std::runtime_error("Not ok!");
        }
    };
    auto all_meshes = mc.all_roots();
    auto ranges = get_meshes(mc, position_attribute_name);
    auto get_mesh_name = [&](int64_t index) -> std::string { return fmt::format("{}", index); };
    int total_F = 0;
    int total_V = 0;
    for (const auto& [name, em] : ranges) {
        total_V = std::max<int>(total_V, em.V.end());
        total_F = std::max<int>(total_F, em.F.end());
        // spdlog::info(
        //     "{}->{} and {}->{} total is {} {}",
        //     em.V.start(),
        //     em.V.end(),
        //     em.F.start(),
        //     em.F.end(),
        //     total_V,
        //     total_F

        //);
    }

    Eigen::MatrixX<double> V(total_V, 3);
    Eigen::MatrixX<int64_t> F(total_F, 3);
    VectorX<int64_t> patch_labels(total_F);
    F.setConstant(-1);
    V.setConstant(-1);
    int64_t index = 0;
    for (auto& [name, em] : ranges) {
        check_degen(name, em.V.M, em.F.M);
        // spdlog::info("Input was {}", name);
        // std::cout << em.V.M << std::endl;
        // spdlog::info("=====");
        // std::cout << em.F.M << std::endl;
        auto F2 = em.F.M.eval();

        em.F.M.array() = em.F.M.array() + em.V.start();
        em.F.assign(F);
        em.V.assign(V);
        patch_labels.segment(em.F.start(), em.F.M.rows()).setConstant(index++);
        em.F.M = F2;
        // spdlog::info("New full face is: ");
        // std::cout << V << std::endl;
        // spdlog::info("=====");
        // std::cout << F << std::endl;
        // igl::writeOBJ(fmt::format("hihi_{}.obj", name), V, F);
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

    check_degen("combo", V, F);

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
    std::map<std::string, std::tuple<int64_t, std::vector<Tuple>>> edge_meshes;


    for (const auto& [inds, pairs] : to_fuse) {
        const auto& [ind_a, ind_b] = inds;


        // spdlog::info("{} {}, {} {}", ind_a, ind_b, get_mesh_name(ind_a), get_mesh_name(ind_b));

        std::string trim_mesh_name = fmt::format("trim_{}_{}", ind_a, ind_b);

        const auto& em_a = ranges.at(get_mesh_name(ind_a));


        const auto& em_b = ranges.at(get_mesh_name(ind_b));


        const Mesh& m = all_meshes.at(get_mesh_name(ind_a));
        std::string edge_mesh_name = fmt::format("trim_{}_{}", ind_a, ind_b);
        {
            std::vector<Tuple> tups;
            tups.reserve(pairs.size());

            for (size_t j = 0; j < pairs.size() - 1; ++j) {
                int64_t a = pairs[j][0];
                int64_t b = pairs[j + 1][0];
                if (a == b) {
                    continue;
                }

                const auto& fa = em_a.VF.at(a);
                const auto& fb = em_a.VF.at(b);
                // spdlog::info(
                //     "pair {} {} got Faces {}: {}, {}: {}",
                //     ind_a,
                //     ind_b,
                //     a,
                //     fmt::join(fa, ","),
                //     b,
                //     fmt::join(fb, ","));

                std::vector<int64_t> fs;
                std::set_intersection(
                    fa.begin(),
                    fa.end(),
                    fb.begin(),
                    fb.end(),
                    std::back_inserter(fs));

                assert(fs.size() == 1); // must be true for a boundary edge

                int64_t fid = fs[0];
                auto f = em_a.F.M.row(fid);
                // spdlog::info("{} | {} => {} {}", fid, fmt::join(f, ","), a, b);
                int8_t lvid = -1;
                int8_t leid = -1;
                for (int k = 0; k < 3; ++k) {
                    if (a == f(k)) {
                        lvid = k;
                    } else if (b != f(k)) {
                        leid = k;
                    }
                }
                Tuple& t = tups.emplace_back(lvid, leid, -1, fid + em_a.V.start());
                // spdlog::info("{} {}", lvid, leid);
                // assert(m.is_valid(t));
            }
            edge_meshes[edge_mesh_name] = std::make_tuple(ind_a, std::move(tups));
        }


        for (auto [ta, tb] : pairs) {
            assert(ta >= 0);
            assert(tb >= 0);
            assert(ta < em_a.V.M.rows());
            assert(tb < em_b.V.M.rows());
            int64_t ota = ta;
            int64_t otb = tb;
            ta += em_a.V.start();
            tb += em_b.V.start();


            assert(em_a.in_v_range(ta));
            assert(em_b.in_v_range(tb));
            // this is too slow to test
            // assert(em_a.in_f_range(ta));
            // assert(em_b.in_f_range(tb));

            assert(ta >= em_a.V.start());
            assert(tb >= em_b.V.start());
            assert(ta < em_a.V.end());
            assert(tb < em_b.V.end());
            // spdlog::info("Merging {} {}", ta, tb);
            Vsets.merge(ta, tb);
        }
    }
    {
        std::vector<size_t> roots = Vsets.roots();
        // spdlog::info("roots: {}", fmt::join(roots, ","));
        std::map<int64_t, int64_t> root_indices;
        for (size_t j = 0; j < roots.size(); ++j) {
            root_indices[roots[j]] = j;
        }

        {
            std::vector<size_t> counts(roots.size());
            for (int j = 0; j < total_V; ++j) {
                counts[root_indices.at(Vsets.get_root(j))]++;
            }
            // for (int j = 0; j < roots.size(); ++j) {
            //     if (counts[j] > 2) {
            //         spdlog::info("Root {} (orig {}) got {}", j, roots[j], counts[j]);
            //     }
            // }
            //  spdlog::info("{}", fmt::join(counts, ","));
            std::vector<size_t> freqs(3);
            for (const auto& freq : counts) {
                if (freq >= freqs.size()) {
                    freqs.resize(freq + 1);
                }
                freqs[freq]++;
            }
            spdlog::info("frequency of frequencies {}", fmt::join(freqs, ","));
            // spdlog::info("{}", fmt::join(counts, ","));
        }


        for (int j = 0; j < F.size(); ++j) {
            int64_t& v = F(j);
            v = root_indices.at(Vsets.get_root(v));
        }
        check_degen("combo", V, F);

        for (const auto& v : roots) {
            assert(v < V.rows());
        }
        V = V(roots, Eigen::all).eval();
        igl::writeOBJ("hi.obj", V, F);
    }


    // spdlog::info("Creating trimesh");
    auto mptr = std::make_shared<wmtk::TriMesh>();

    mptr->initialize(F);
    wmtk::mesh_utils::set_matrix_attribute(
        V,
        std::string(position_attribute_name),
        wmtk::PrimitiveType::Vertex,
        *mptr);


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
    VectorX<int64_t> edge_labels(total_edges);

    size_t current_vertex_size = 0;
    size_t current_edge_size = 0;
    std::vector<std::array<Tuple, 2>> map(total_edges);
    index = 0;
    for (const auto& [name, pr] : edge_meshes) {
        const auto& [mesh_id, tups] = pr;
        const auto& mesh_a = all_meshes.at(get_mesh_name(mesh_id));
        const auto& range_a = ranges.at(get_mesh_name(mesh_id));

        patch_labels.segment(current_edge_size, tups.size()).setConstant(index++);

        auto EB = E.block(current_edge_size, 0, tups.size(), 2);
        for (int k = 0; k < tups.size(); ++k) {
            EB.row(k) << current_vertex_size + k, current_vertex_size + k + 1;
        }
        for (size_t j = 0; j < tups.size(); ++j) {
            size_t global_index = current_edge_size + j;
            auto& m = map[global_index];

            m[0] = Tuple(0, -1, -1, global_index);
            m[1] = patch_mesh->map_to_parent_tuple(simplex::Simplex(PrimitiveType::Edge, tups[j]));
        }
        current_vertex_size += tups.size() + 1;
        current_edge_size += tups.size();
    }
    auto em_ptr = std::make_shared<wmtk::EdgeMesh>();
    em_ptr->initialize(E);

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
