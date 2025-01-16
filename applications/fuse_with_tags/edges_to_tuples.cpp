#include "edges_to_tuples.hpp"
#include <wmtk/TriMesh.hpp>
using namespace wmtk;
std::vector<Tuple> boundary_edges_to_tuples(
    const wmtk::TriMesh& m,
    const std::vector<int64_t>& indices)
{
    std::vector<Tuple> tups;
    tups.reserve(indices.size());

    auto handle = m.get_attribute_handle<int64_t>("m_fv", wmtk::PrimitiveType::Triangle);
    auto acc = m.create_const_accessor<int64_t>(handle);
    for (size_t j = 0; j < indices.size() - 1; ++j) {
        int64_t a = indices[j];
        int64_t b = indices[j + 1];
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
        std::set_intersection(fa.begin(), fa.end(), fb.begin(), fb.end(), std::back_inserter(fs));

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
        std::cout << (acc.const_vector_attribute(Tuple(lvid, leid, -1, fid))).transpose()
                  << std::endl;
        assert(
            acc.const_vector_attribute(Tuple(lvid, leid, -1, fid)) ==
            V.row(em_a.V.start() + a).transpose());
        // assert(acc.const_vector_attribute(Tuple(3 -lvid - leid, leid, -1, fid)) ==
        // V.row(em_b.V.start() + b).transpose());
        // assert(patch_acc.const_vector_attribute(t) == V.row(em_a.V.start() +
        // a).transpose());
        // assert(patch_acc.const_vector_attribute(patch_mesh->switch_vertex(t)) ==
        // V.row(em_b.V.start() + b).transpose());
        //  spdlog::info("{} {}", lvid, leid);
        //  assert(m.is_valid(t));
    }
    edge_meshes[edge_mesh_name] = std::make_tuple(ind_a, std::move(tups));
    fuse_names_ordered.emplace_back(edge_mesh_name);
}
}
