#include "TetImplicitsMesh.h"

#include "wmtk/utils/Rational.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/predicates/predicates.h>
#include <igl/Timer.h>
#include <igl/orientable_patches.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <paraviewo/VTUWriter.hpp>

namespace {
static int debug_print_counter = 0;
}

namespace wmtk::components::tet_implicits {


VertexAttributes::VertexAttributes(const Vector3d& p)
    : m_posf(p)
{}

double TetImplicitsMesh::get_length2(const Tuple& l) const
{
    SmartTuple v1(*this, l);
    SmartTuple v2 = v1.switch_vertex();
    double length =
        (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf).squaredNorm();
    return length;
}

Vector3d TetImplicitsMesh::tet_center(const size_t tid) const
{
    const auto vs = oriented_tet_vids(tid);
    const Vector3d p = 0.25 * (m_vertex_attribute[vs[0]].m_posf + m_vertex_attribute[vs[1]].m_posf +
                               m_vertex_attribute[vs[2]].m_posf + m_vertex_attribute[vs[3]].m_posf);
    return p;
}

bool TetImplicitsMesh::tet_is_inside(const size_t tid) const
{
    const auto vs = oriented_tet_vids(tid);

    if (m_vertex_attribute[vs[0]].m_is_inside && m_vertex_attribute[vs[1]].m_is_inside &&
        m_vertex_attribute[vs[2]].m_is_inside && m_vertex_attribute[vs[3]].m_is_inside) {
        return true;
    }
    return false;
}

void TetImplicitsMesh::write_msh(std::string file)
{
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return m_vertex_attribute[i].m_posf;
    });

    const auto& tets = get_tets();
    msh.add_tets(tets.size(), [&](size_t k) {
        auto i = tets[k].tid(*this);
        auto vs = oriented_tet_vertices(tets[k]);
        std::array<size_t, 4> data;
        for (int j = 0; j < 4; j++) {
            data[j] = vs[j].vid(*this);
            assert(data[j] < vtx.size());
        }
        return data;
    });

    msh.add_tet_attribute<1>("t energy", [&](size_t i) {
        return std::cbrt(m_tet_attribute[i].m_quality);
    });

    for (size_t j = 0; j < m_tags_count; ++j) {
        msh.add_tet_attribute<1>(fmt::format("tag_{}", j), [&](size_t i) {
            return m_tet_attribute[i].tags[j];
        });
    }

    msh.add_physical_group("ImageVolume");

    // msh.add_face_vertices(m_V_envelope.size(), [this](size_t k) { return m_V_envelope[k]; });
    // msh.add_faces(m_F_envelope.size(), [this](size_t k) { return m_F_envelope[k]; });
    // msh.add_physical_group("EnvelopeSurface");

    msh.save(file, true);
}

std::tuple<double, double> TetImplicitsMesh::get_max_avg_energy()
{
    double max_energy = -1.;
    double avg_energy = 0.;
    size_t cnt = 0;

    for (int i = 0; i < tet_capacity(); i++) {
        auto tup = tuple_from_tet(i);
        if (!tup.is_valid(*this)) {
            continue;
        }

        const double& q = m_tet_attribute[tup.tid(*this)].m_quality;
        max_energy = std::max(max_energy, q);
        avg_energy += std::cbrt(q);
        cnt++;
    }

    avg_energy /= cnt;

    return std::make_tuple(std::cbrt(max_energy), avg_energy);
}

bool TetImplicitsMesh::is_inverted(const std::array<size_t, 4>& vs) const
{
    // Return a positive value if the point pd lies below the
    // plane passing through pa, pb, and pc; "below" is defined so
    // that pa, pb, and pc appear in counterclockwise order when
    // viewed from above the plane.

    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(
        m_vertex_attribute[vs[0]].m_posf,
        m_vertex_attribute[vs[1]].m_posf,
        m_vertex_attribute[vs[2]].m_posf,
        m_vertex_attribute[vs[3]].m_posf);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
        return false;
    return true;
}

bool TetImplicitsMesh::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tet_vids(loc);
    return is_inverted(vs);
}

double TetImplicitsMesh::get_quality(const std::array<size_t, 4>& its) const
{
    std::array<Vector3d, 4> ps;
    for (int k = 0; k < 4; k++) {
        ps[k] = m_vertex_attribute[its[k]].m_posf;
    }
    double energy = -1.;
    {
        std::array<double, 12> T;
        for (int k = 0; k < 4; k++)
            for (int j = 0; j < 3; j++) T[k * 3 + j] = ps[k][j];

        energy = wmtk::AMIPS_energy_stable_p3<wmtk::Rational>(T);
    }
    if (std::isinf(energy) || std::isnan(energy) || energy < 27 - 1e-3) return MAX_ENERGY;
    return energy;
}

double TetImplicitsMesh::get_quality(const Tuple& loc) const
{
    auto its = oriented_tet_vids(loc);
    return get_quality(its);
}

// std::vector<std::array<size_t, 3>> TetImplicitsMesh::get_faces_by_condition(
//     std::function<bool(const FaceAttributes&)> cond)
//{
//     auto res = std::vector<std::array<size_t, 3>>();
//     for (auto f : get_faces()) {
//         auto fid = f.fid(*this);
//         if (cond(m_face_attribute[fid])) {
//            auto tid = fid / 4, lid = fid % 4;
//            auto verts = get_face_vertices(f);
//            res.emplace_back(std::array<size_t, 3>{
//                {verts[0].vid(*this), verts[1].vid(*this), verts[2].vid(*this)}});
//        }
//    }
//    return res;
//}

bool TetImplicitsMesh::check_attributes()
{
    // check quality
    const auto& tets = get_tets();
    for (const auto& t : tets) {
        size_t i = t.tid(*this);
        double q = get_quality(t);
        if (q != m_tet_attribute[i].m_quality) {
            wmtk::logger().critical(
                "q!=m_tet_attribute[i].m_quality {} {}",
                q,
                m_tet_attribute[i].m_quality);
            return false;
        }
    }
    return true;
}

void TetImplicitsMesh::op_separate()
{
    if (m_params.input_tags.size() != 2) {
        log_and_throw_error("input_tags must have size 2, input_tags is {}", m_params.input_tags);
    }
    if (m_params.output_tags.size() != 1) {
        log_and_throw_error(
            "output_tags must have size 1, output_tags is {}",
            m_params.output_tags);
    }

    auto& bvh0 = m_bvh[m_params.input_tags[0]];
    auto& bvh1 = m_bvh[m_params.input_tags[1]];

    m_sq_sdf = [this, bvh0, bvh1](const Vector3d& p) -> double {
        Vector3d nearest;
        double sq_dist0;
        bvh0->nearest_facet(p, nearest, sq_dist0);
        double sq_dist1;
        bvh1->nearest_facet(p, nearest, sq_dist1);
        double sq_dist = std::max(sq_dist0, sq_dist1);
        return sq_dist;
    };

    m_new_tag = [this](const Vector3d&) { return m_params.output_tags[0]; };

    compute();

    m_sq_sdf = nullptr;
    m_new_tag = nullptr;
}

void TetImplicitsMesh::op_tight_seal()
{
    if (m_params.input_tags.size() != 2) {
        log_and_throw_error("input_tags must have size 2, input_tags is {}", m_params.input_tags);
    }
    if (m_params.output_tags.size() != 0) {
        log_and_throw_error(
            "output_tags must have size 0, output_tags is {}",
            m_params.output_tags);
    }

    // add temporary tag ID to store intermediate result in
    for (const Tuple& t : get_tets()) {
        m_tet_attribute[t.tid(*this)].tags.push_back(0);
    }
    const auto& input_tags = m_params.input_tags;

    auto& bvh0 = m_bvh[input_tags[0]];
    auto& bvh1 = m_bvh[input_tags[1]];

    // phase 1: find region of interest

    m_sq_sdf = [this, bvh0, bvh1](const Vector3d& p) -> double {
        Vector3d nearest;
        double sq_dist0;
        bvh0->nearest_facet(p, nearest, sq_dist0);
        double sq_dist1;
        bvh1->nearest_facet(p, nearest, sq_dist1);
        double dist0 = std::sqrt(sq_dist0);
        double dist1 = std::sqrt(sq_dist1);
        double dist = dist0 + dist1;
        return dist * dist;
    };

    // compute
    {
        // mark vertices as inside / outside
        for (const Tuple& t : get_vertices()) {
            auto& attrs = m_vertex_attribute[t.vid(*this)];
            double d2 = m_sq_sdf(attrs.m_posf);
            // also consider equal as inside
            attrs.m_sq_sdf = d2;
            attrs.m_is_inside = (d2 <= m_params.d2);
        }

        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", debug_print_counter++));
        }

        // split edges in between inside/outide labels (marching tets)
        std::vector<simplex::Edge> split_edges;
        for (const Tuple& t : get_edges()) {
            size_t v0 = t.vid(*this);
            size_t v1 = t.switch_vertex(*this).vid(*this);

            const auto& VA = m_vertex_attribute;
            if (!(VA[v0].m_is_inside ^ VA[v1].m_is_inside)) {
                continue;
            }

            // only split edges that are not in the input_tag regions
            const auto tets = get_incident_tids_for_edge(t);
            bool has_outside = false;
            for (const size_t tid : tets) {
                const auto& tags = m_tet_attribute.at(tid).tags;
                if ((tags[input_tags[0].first] != input_tags[0].second) &&
                    (tags[input_tags[1].first] != input_tags[1].second)) {
                    has_outside = true;
                    break;
                }
            }
            if (!has_outside) {
                continue;
            }

            split_edges.emplace_back(v0, v1);
        }

        for (const auto& e : split_edges) {
            const Tuple& t = tuple_from_edge(e.vertices());
            std::vector<Tuple> new_tets;
            split_edge(t, new_tets);
        }

        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", debug_print_counter++));
        }

        for (const Tuple& t : get_tets()) {
            const size_t tid = t.tid(*this);
            auto& tags = m_tet_attribute[tid].tags;
            if ((tags[input_tags[0].first] == input_tags[0].second) ||
                (tags[input_tags[1].first] == input_tags[1].second)) {
                continue;
            }
            if (tet_is_inside(tid)) {
                tags[m_tags_count] = 1;
            }
        }

        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", debug_print_counter++));
        }
    }

    // phase 2: find region that is closer to second tag

    m_sq_sdf = [this, bvh0, bvh1](const Vector3d& p) -> double {
        Vector3d nearest;
        double sq_dist0;
        bvh0->nearest_facet(p, nearest, sq_dist0);
        double sq_dist1;
        bvh1->nearest_facet(p, nearest, sq_dist1);
        double sq_dist = sq_dist0 - sq_dist1;
        return sq_dist;
    };

    // compute
    const double d2_actual = m_params.d2;
    m_params.d2 = 0;
    {
        // mark vertices as inside / outside
        for (const Tuple& t : get_vertices()) {
            auto& attrs = m_vertex_attribute[t.vid(*this)];
            double d2 = m_sq_sdf(attrs.m_posf);
            // also consider equal as inside
            attrs.m_sq_sdf = d2;
            attrs.m_is_inside = (d2 <= m_params.d2);
        }

        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", debug_print_counter++));
        }

        // split edges in between inside/outide labels (marching tets)
        std::vector<simplex::Edge> split_edges;
        for (const Tuple& t : get_edges()) {
            size_t v0 = t.vid(*this);
            size_t v1 = t.switch_vertex(*this).vid(*this);

            const auto& VA = m_vertex_attribute;
            if (!(VA[v0].m_is_inside ^ VA[v1].m_is_inside)) {
                continue;
            }

            // only split edges that are within the filter
            const auto tets = get_incident_tids_for_edge(t);
            bool is_inside = false;
            for (const size_t tid : tets) {
                const auto& tags = m_tet_attribute.at(tid).tags;
                if ((tags[m_tags_count] == 1)) {
                    is_inside = true;
                    break;
                }
            }
            if (!is_inside) {
                continue;
            }

            split_edges.emplace_back(v0, v1);
        }

        for (const auto& e : split_edges) {
            const Tuple& t = tuple_from_edge(e.vertices());
            std::vector<Tuple> new_tets;
            split_edge(t, new_tets);
        }

        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", debug_print_counter++));
        }

        for (const Tuple& t : get_tets()) {
            const size_t tid = t.tid(*this);
            if (m_tet_attribute[tid].tags[m_tags_count] != 1) {
                continue;
            }

            if (tet_is_inside(tid)) {
                m_tet_attribute[tid].tags[input_tags[0].first] = input_tags[0].second;
            } else {
                m_tet_attribute[tid].tags[input_tags[1].first] = input_tags[1].second;
            }
        }

        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", debug_print_counter++));
        }
    }
    m_params.d2 = d2_actual;

    // phase 3: clean up

    m_sq_sdf = nullptr;
    m_new_tag = nullptr;

    // remove temporary tag ID
    for (const Tuple& t : get_tets()) {
        m_tet_attribute[t.tid(*this)].tags.pop_back();
    }

    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", debug_print_counter++));
    }
}

void TetImplicitsMesh::compute()
{
    if (m_sq_sdf == nullptr || m_new_tag == nullptr) {
        log_and_throw_error("No operation was set. Cannot compute.");
    }

    // mark vertices as inside / outside
    for (const Tuple& t : get_vertices()) {
        auto& attrs = m_vertex_attribute[t.vid(*this)];
        double d2 = m_sq_sdf(attrs.m_posf);
        // also consider equal as inside
        attrs.m_sq_sdf = d2;
        attrs.m_is_inside = (d2 <= m_params.d2);
    }

    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", debug_print_counter++));
    }

    // split edges in between inside/outide labels (marching tets)
    std::vector<simplex::Edge> split_edges;
    for (const Tuple& t : get_edges()) {
        size_t v0 = t.vid(*this);
        size_t v1 = t.switch_vertex(*this).vid(*this);

        const auto& VA = m_vertex_attribute;
        if (VA[v0].m_is_inside ^ VA[v1].m_is_inside) {
            split_edges.emplace_back(v0, v1);
        }
    }

    for (const auto& e : split_edges) {
        const Tuple& t = tuple_from_edge(e.vertices());
        std::vector<Tuple> new_tets;
        split_edge(t, new_tets);
    }

    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", debug_print_counter++));
    }

    // tag inside-tets with output_tags
    // an inside-tet is one with all vertices being inside
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (tet_is_inside(tid)) {
            const auto tag = m_new_tag(tet_center(tid));
            m_tet_attribute[tid].tags[tag.first] = tag.second;
        }
    }

    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", debug_print_counter++));
    }
}

// util functions for union find
int find_uf(int v, std::vector<int>& parent)
{
    int root = v;
    while (parent[root] != root) {
        root = parent[root];
    }
    // path compression optimization
    while (parent[v] != root) {
        int next = parent[v];
        parent[v] = root;
        v = next;
    }
    return root;
}

void union_uf(int u, int v, std::vector<int>& parent)
{
    int root_u = find_uf(u, parent);
    int root_v = find_uf(v, parent);
    if (root_u != root_v) {
        parent[root_u] = root_v;
    }
}

void TetImplicitsMesh::write_vtu(const std::string& path)
{
    // consolidate_mesh();
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);
    const auto& vs = get_vertices();
    const auto& tets = get_tets();
    // const auto faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });

    Eigen::MatrixXd V(vert_capacity(), 3);
    Eigen::MatrixXi T(tet_capacity(), 4);
    // Eigen::MatrixXi F(faces.size(), 3);

    V.setZero();
    T.setZero();
    // F.setZero();

    VectorXd v_sdf(vert_capacity());
    v_sdf.setZero();
    VectorXd v_is_inside(vert_capacity());
    v_is_inside.setZero();

    size_t tags_count = m_tags_count;
    if (!tets.empty()) {
        tags_count = m_tet_attribute[tets[0].tid(*this)].tags.size();
    }

    std::vector<VectorXd> tags(tags_count, VectorXd(tet_capacity()));
    for (auto& t : tags) {
        t.setZero();
    }
    VectorXd amips(tet_capacity());
    amips.setZero();

    int index = 0;
    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        for (size_t j = 0; j < tags_count; ++j) {
            tags[j][index] = m_tet_attribute[tid].tags[j];
        }
        amips[index] = std::cbrt(m_tet_attribute[tid].m_quality);

        const auto& vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            T(index, j) = vs[j].vid(*this);
        }
        ++index;
    }

    // for (size_t i = 0; i < faces.size(); ++i) {
    //     for (size_t j = 0; j < 3; ++j) {
    //         F(i, j) = faces[i][j];
    //     }
    // }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;

        const double sq_sdf = m_vertex_attribute[vid].m_sq_sdf;
        v_sdf(vid) = std::sqrt(std::abs(sq_sdf));
        if (std::signbit(sq_sdf)) {
            v_sdf(vid) *= -1;
        }
        v_is_inside(vid) = m_vertex_attribute[vid].m_is_inside;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();

    for (size_t j = 0; j < tags_count; ++j) {
        writer->add_cell_field(fmt::format("tag_{}", j), tags[j]);
    }
    writer->add_cell_field("quality", amips);
    writer->add_field("sdf", v_sdf);
    writer->add_field("is_inside", v_is_inside);
    writer->write_mesh(path + ".vtu", V, T);

    //// surface
    //{
    //    const auto surf_out_path = path + "_surf.vtu";
    //    std::shared_ptr<paraviewo::ParaviewWriter> surf_writer;
    //    surf_writer = std::make_shared<paraviewo::VTUWriter>();
    //    surf_writer->add_field("sizing_field", v_sizing_field);
    //
    //    logger().info("Write {}", surf_out_path);
    //    surf_writer->write_mesh(surf_out_path, V, F);
    //}
}

void TetImplicitsMesh::write_surface(const std::string& path) const
{
    log_and_throw_error("write_surface is not implemented / commented out");
    // std::vector<std::array<size_t, 3>> outface;
    // for (const Tuple& f : get_faces()) {
    //     if (!m_face_attribute[f.fid(*this)].m_is_surface_fs) {
    //         continue;
    //     }
    //     const auto verts = get_face_vertices(f);
    //     std::array<size_t, 3> vids = {
    //         {verts[0].vid(*this), verts[1].vid(*this), verts[2].vid(*this)}};
    //     outface.emplace_back(vids);
    // }
    // Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    // for (const Tuple& v : get_vertices()) {
    //     const size_t vid = v.vid(*this);
    //     matV.row(vid) = m_vertex_attribute[vid].m_posf;
    // }
    // Eigen::MatrixXi matF(outface.size(), 3);
    // for (size_t i = 0; i < outface.size(); i++) {
    //     matF.row(i) << outface[i][0], outface[i][1], outface[i][2];
    // }
    // igl::write_triangle_mesh(path, matV, matF);
    //
    // wmtk::logger().info("Output face size {}", outface.size());
}

} // namespace wmtk::components::tet_implicits