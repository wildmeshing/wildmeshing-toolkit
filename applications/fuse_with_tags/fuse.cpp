#include "fuse.hpp"
#include <fmt/format.h>
#include <set>
#include <fmt/ranges.h>
#include <igl/writeOBJ.h>
#include <spdlog/spdlog.h>
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

std::shared_ptr<wmtk::TriMesh> fuse(
    wmtk::components::multimesh::MeshCollection& mc,
    const std::map<std::array<int64_t, 2>, std::vector<std::array<int64_t, 2>>>& to_fuse,
    // const std::map<std::array<int64_t, 2>, std::vector<std::array<Tuple, 2>>>& to_fuse,
    const std::string_view& position_attribute_name)
{

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
                        spdlog::info("Fail on mesh {}, face {}:  {}", name, j, fmt::join(f,","));
                        ok = false;
                    }
                }
            }
        }
        if(!ok) {
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
    F.setConstant(-1);
    V.setConstant(-1);
    for (auto& [name, em] : ranges) {
        check_degen(name, em.V.M,em.F.M);
        // spdlog::info("Input was {}", name);
        // std::cout << em.V.M << std::endl;
        // spdlog::info("=====");
        // std::cout << em.F.M << std::endl;
        em.F.M.array() = em.F.M.array() + em.V.start();
        em.F.assign(F);
        em.V.assign(V);
        // spdlog::info("New full face is: ");
        // std::cout << V << std::endl;
        // spdlog::info("=====");
        // std::cout << F << std::endl;
        // igl::writeOBJ(fmt::format("hihi_{}.obj", name), V, F);
    }
     igl::writeOBJ("hihi.obj", V, F);

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

    check_degen("combo",V,F);

     for (const auto& [n, a] : ranges) {
         spdlog::info("Name: {}", n);
     }
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

    for (const auto& [inds, pairs] : to_fuse) {
        const auto& [ind_a, ind_b] = inds;
        std::ofstream ofs(fmt::format("pairs_{}_{}.txt", ind_a,ind_b));

        // spdlog::info("{} {}, {} {}", ind_a, ind_b, get_mesh_name(ind_a), get_mesh_name(ind_b));

        const auto& em_a = ranges.at(get_mesh_name(ind_a));
        const auto& em_b = ranges.at(get_mesh_name(ind_b));

        std::set<int64_t> bad = {22507,218824 , 47400, 218817};
        auto contains_bad = [&](int64_t x) {
            return bad.find(x) != bad.end();
        };
        for (auto [ta, tb] : pairs) {
            ofs << ta << " " << tb << "\n";
            assert(ta >= 0);
            assert(tb >= 0);
            assert(ta < em_a.V.M.rows());
            assert(tb < em_b.V.M.rows());
            int64_t ota = ta;
            int64_t otb = tb;
            ta += em_a.V.start();
            tb += em_b.V.start();

            if(contains_bad(ta) || contains_bad(ta)) {
                spdlog::info("meshes:{}, {}->{} {}->{}", fmt::join(inds,","), ota,ta,otb,tb);
            }

            assert(em_a.in_v_range(ta));
            assert(em_b.in_v_range(tb));
            // this is too slow to test
            //assert(em_a.in_f_range(ta));
            //assert(em_b.in_f_range(tb));

            assert(ta >= em_a.V.start());
            assert(tb >= em_b.V.start());
            assert(ta < em_a.V.end());
            assert(tb < em_b.V.end());
            // spdlog::info("Merging {} {}", ta, tb);
            Vsets.merge(ta, tb);
            if(contains_bad(Vsets.get_root(ta)) || contains_bad(Vsets.get_root(tb))) {
                spdlog::info("post merge got bad:{}, {}->{} {}->{}", fmt::join(inds,","), ota,ta,otb,tb);
            }
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
            for (int j = 0; j < roots.size(); ++j) {
                if(counts[j] > 2) {
                    spdlog::info("Root {} (orig {}) got {}", j, roots[j], counts[j]);
                }
            }
            // spdlog::info("{}", fmt::join(counts, ","));
            std::vector<size_t> freqs(3);
            for(const auto& freq: counts) {
                if(freq >= freqs.size()) {
                    freqs.resize(freq+1);
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
        igl::writeOBJ("hi.obj", V, F);
        check_degen("combo",V,F);

        for (const auto& v : roots) {
            assert(v < V.rows());
        }
        V = V(roots, Eigen::all).eval();
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
    for (const auto& [name, em] : ranges) {
        // spdlog::info("Registering {} as child mesh", name);
        auto& m = const_cast<wmtk::Mesh&>(all_meshes.at(name));
        std::vector<int64_t> i(em.F.M.rows());
        assert(em.F.M.rows() == m.get_all(wmtk::PrimitiveType::Triangle).size());
        std::iota(i.begin(), i.end(), em.F.start());

        wmtk::components::multimesh::from_facet_surjection(*mptr, m, i);
    }

    return mptr;
}
