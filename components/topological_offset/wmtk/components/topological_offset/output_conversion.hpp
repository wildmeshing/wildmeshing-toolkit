#pragma once
#include <wmtk/TetMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetTetMesh.h>
#include <algorithm>
#include <type_traits>
#include <wmtk/io/tagged_mesh_data.hpp>

namespace wmtk::components::topological_offset {
/// The edge block and the point block of a mesh hold_curves_and_points() has not held -- an
/// empty input complex returns before it (topological_offset.cpp) -- from the tags as loaded
/// (EdgeAttributes::feature_inputs, VertexExtra::point_inputs), as io::collect_feature_edge_tags()
/// and io::collect_point_tags() write them from the engine's: rows of the vertices in
/// get_vertices() order, an edge's vertices ascending, one row per vertex under all its labels.
/// Until 2026-11 that exit wrote neither block, and the input's curves and points were dropped.
inline void collect_loaded_curves_and_points(const TopoOffsetTetMesh& mesh, MeshMap& outputs)
{
    std::vector<int64_t> row(mesh.vert_capacity(), -1);
    int64_t n = 0;
    for (const auto& v : mesh.get_vertices()) row[v.vid(mesh)] = n++;
    // Each (row's vertices, set) as a block of `type`, one tags column per name.
    const auto write = [&](const CellType type,
                           const std::vector<std::vector<int64_t>>& rows,
                           const std::vector<InputSet>& sets,
                           const std::vector<std::string>& names) {
        auto& block = outputs.at("mesh").cells[type];
        block.connectivity.resize(Eigen::Index(rows.size()), cell_size(type));
        DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(rows.size(), names.size());
        for (size_t i = 0; i < rows.size(); ++i) {
            for (size_t k = 0; k < rows[i].size(); ++k)
                block.connectivity(Eigen::Index(i), Eigen::Index(k)) = rows[i][k];
            for (size_t j = 0; j < names.size(); ++j)
                member(Eigen::Index(i), Eigen::Index(j)) = (sets[i] >> j) & 1;
        }
        block.attributes["tags"] = {std::move(member), names};
    };
    std::vector<std::vector<int64_t>> rows;
    std::vector<InputSet> sets;
    if (!mesh.m_feature_edge_names.empty()) {
        for (const auto& e : mesh.get_edges()) {
            const InputSet set = mesh.m_edge_attribute[e.eid(mesh)].feature_inputs;
            if (set == 0) continue;
            const size_t a = e.vid(mesh), b = e.switch_vertex(mesh).vid(mesh);
            rows.push_back({row[std::min(a, b)], row[std::max(a, b)]});
            sets.push_back(set);
        }
        write(CellType::Edge, rows, sets, mesh.m_feature_edge_names);
    }
    rows.clear();
    sets.clear();
    if (!mesh.m_point_names.empty()) {
        for (const auto& v : mesh.get_vertices()) {
            const InputSet set = mesh.m_vertex_extra[v.vid(mesh)].point_inputs;
            if (set == 0) continue;
            rows.push_back({row[v.vid(mesh)]});
            sets.push_back(set);
        }
        write(CellType::Point, rows, sets, mesh.m_point_names);
    }
}

template <typename Mesh>
void collect_memory_output(Mesh& mesh, io::ComponentIO& context)
{
    MeshMap raw;
    // The result: the tags, the data fields and two annotations (write_vtu_frame()). Its companion
    // exports -- the input complex, the front, their edges; in 2D `_surf` is the envelope curve,
    // returned as `envelope` below -- are the mesh's tags, and io::default_output_keys() drops
    // them. No stencil file, a debug-frame diagnostic indexed by its own points, not by the mesh's
    // vertex slots, which compact_slot_outputs() could not remap ("Export references an inactive
    // vertex": every run of the 2026-10-29 sonic sweep aborted there).
    mesh.write_vtu("mesh", &raw, /*result=*/true);
    if constexpr (std::is_base_of_v<TetMesh, Mesh>) io::compact_slot_outputs(mesh, raw);
    context.result.first = io::named_outputs(std::move(raw));
    auto& data = context.result.first.at("mesh");
    if constexpr (std::is_base_of_v<TetMesh, Mesh>) {
        io::collect_tags(
            mesh,
            context.result.first,
            CellType::Tetrahedron,
            mesh.get_tets(),
            [&](const auto& t) -> const auto& { return mesh.m_tet_attribute[t.tid(mesh)].tag; });
        io::collect_input_set_tags<CellType::Triangle>(
            mesh,
            context.result.first,
            mesh.m_face_tag_names);
        if (mesh.m_edge_tags_held) {
            // the edge tags, as the engine's feature tags carry them (hold_curves_and_points()),
            // the edge block tetwild and simwild write; a tag-expression curve's bit, past the
            // names, is not written
            io::collect_feature_edge_tags(mesh, context.result.first, mesh.m_feature_edge_names);
            // the input points, as their anchors carry them (hold_curves_and_points()), the point
            // block tetwild and simwild write
            io::collect_point_tags(mesh, context.result.first, mesh.m_point_names);
        } else {
            collect_loaded_curves_and_points(mesh, context.result.first);
        }
    } else {
        io::collect_tags(
            mesh,
            context.result.first,
            CellType::Triangle,
            mesh.get_faces(),
            [&](const auto& t) -> const auto& { return mesh.m_face_attribute[t.fid(mesh)].tags; });
    }
    if (mesh.m_offset_params.save_offset_correspondence) {
        const auto ids = data.vertex_attributes.at("vid").integers();
        DataMatrix<int64_t> correspondence(data.V.rows(), 1);
        for (Eigen::Index i = 0; i < ids.rows(); ++i)
            correspondence(i, 0) = mesh.m_vertex_extra[ids(i, 0)].m_corr_input_vid;
        data.vertex_attributes["corr_input_vid"].values = std::move(correspondence);
        data.vertex_attributes["vid"].values = ids;
    }
    if (mesh.m_has_envelope) {
        context.result.first["envelope"] = io::geometry_data(
            mesh.m_V_envelope,
            mesh.m_F_envelope,
            std::is_base_of_v<TetMesh, Mesh> ? CellType::Triangle : CellType::Edge);
    }
    data.validate();
}
/// The band's boundary as rows to hand on with a reference: its triangles on vertices of their
/// own, and per row the labels the output gives the regions it separates.
struct OffsetRows
{
    MatrixXd V = MatrixXd(0, 3);
    MatrixXi F = MatrixXi(0, 3);
    std::vector<std::vector<std::string>> names;
};

/// The band's boundary -- every face between a band cell and a cell outside the band, the domain
/// wall's excepted: the final offset surface and, where the band wraps a region or takes its
/// cells, its interface with that region -- each named by the tet tags that differ across it as
/// the output labels them (io::tag_labels()), the labels the next component holds it under
/// (TopoOffsetTetMesh::assign_face_labels(), simwild's alike). The whole boundary, as every other
/// label's envelope covers its region's. With the offset surface alone, simwild found the faces
/// between a region and its band outside the band's envelope (72 on a unit tet, 2026-11-05); with
/// the band's labels alone, the next offset found the boundary a band moves into a region it takes
/// cells of outside that region's envelope (118, feature demo 4's cube, 2026-11-05).
inline OffsetRows offset_rows(const TopoOffsetTetMesh& mesh)
{
    OffsetRows out;
    const std::vector<std::string> labels =
        io::tag_labels(mesh.m_tag_id_to_name, mesh.m_tags_count);
    std::vector<int> row(mesh.vert_capacity(), -1);
    std::vector<Vector3d> V;
    std::vector<std::array<int, 3>> F;
    for (const TetMesh::Tuple& f : mesh.get_faces()) {
        const auto opp = f.switch_tetrahedron(mesh);
        if (!opp ||
            mesh.cell_is_offset_band(f.tid(mesh)) == mesh.cell_is_offset_band(opp->tid(mesh)))
            continue;
        const auto& a = mesh.m_tet_attribute[f.tid(mesh)].tag;
        const auto& b = mesh.m_tet_attribute[opp->tid(mesh)].tag;
        std::vector<std::string> names;
        for (const auto& [x, y] : {std::pair(&a, &b), std::pair(&b, &a)}) {
            for (const int64_t t : *x)
                if (t > 0 && !y->count(t) && size_t(t) < labels.size())
                    names.push_back(labels[size_t(t)]); // ambient carries no label
        }
        if (names.empty()) continue;
        out.names.push_back(std::move(names));
        std::array<int, 3> r;
        const auto vs = mesh.get_face_vids(f);
        for (int k = 0; k < 3; ++k) {
            int& v = row[vs[k]];
            if (v < 0) {
                v = int(V.size());
                V.push_back(mesh.m_vertex_attribute[vs[k]].m_posf);
            }
            r[k] = v;
        }
        F.push_back(r);
    }
    out.V.resize(Eigen::Index(V.size()), 3);
    for (size_t i = 0; i < V.size(); ++i) out.V.row(Eigen::Index(i)) = V[i].transpose();
    out.F.resize(Eigen::Index(F.size()), 3);
    for (size_t i = 0; i < F.size(); ++i)
        for (int k = 0; k < 3; ++k) out.F(Eigen::Index(i), k) = F[i][k];
    return out;
}

/// `reference`, a labelled reference as given, with `offset`'s triangles added to its triangle
/// block, each labelled by its names in offset.names, its own rows keeping theirs: built anew
/// with the io helpers the producers build theirs with (io::append_envelope_block(),
/// io::append_envelope_points()), each block on vertices of its own.
inline MeshData with_offset_surface(const MeshData& reference, const OffsetRows& offset)
{
    struct Block
    {
        MatrixXd V = MatrixXd(0, 3);
        MatrixXi C;
        std::vector<InputSet> sets;
        std::vector<std::string> names;
    };
    // The `type` rows of the reference on vertices of their own, in first use, with their labels.
    const auto block = [&](const CellType type) {
        Block b;
        const auto it = reference.cells.find(type);
        if (it == reference.cells.end()) return b;
        const auto& C = it->second.connectivity;
        std::vector<int> row(size_t(reference.V.rows()), -1);
        std::vector<Eigen::Index> used;
        b.C.resize(C.rows(), C.cols());
        for (Eigen::Index r = 0; r < C.rows(); ++r) {
            for (Eigen::Index k = 0; k < C.cols(); ++k) {
                int& v = row[size_t(C(r, k))];
                if (v < 0) {
                    v = int(used.size());
                    used.push_back(Eigen::Index(C(r, k)));
                }
                b.C(r, k) = v;
            }
        }
        b.V.resize(Eigen::Index(used.size()), 3);
        for (size_t i = 0; i < used.size(); ++i)
            b.V.row(Eigen::Index(i)) = reference.V.row(used[i]);
        const auto tags = it->second.attributes.find("tags");
        if (tags == it->second.attributes.end()) return b;
        b.names = tags->second.labels;
        const auto member = tags->second.integers();
        b.sets.assign(size_t(C.rows()), 0);
        for (Eigen::Index r = 0; r < C.rows(); ++r) {
            for (size_t j = 0; j < b.names.size() && j < size_t(SurfaceTagAttributes::max_inputs);
                 ++j) {
                if (member(r, Eigen::Index(j))) b.sets[size_t(r)] |= InputSet(1) << j;
            }
        }
        return b;
    };
    Block tri = block(CellType::Triangle);
    tri.sets.resize(size_t(tri.C.rows()), 0);
    for (const auto& names : offset.names) {
        InputSet set = 0;
        for (const std::string& name : names) {
            auto k = std::find(tri.names.begin(), tri.names.end(), name);
            if (k == tri.names.end()) k = tri.names.insert(tri.names.end(), name);
            const size_t j = size_t(k - tri.names.begin());
            if (j < size_t(SurfaceTagAttributes::max_inputs)) set |= InputSet(1) << j;
        }
        tri.sets.push_back(set);
    }
    const Eigen::Index nv = tri.V.rows(), nf = tri.C.rows();
    tri.V.conservativeResize(nv + offset.V.rows(), 3);
    tri.V.bottomRows(offset.V.rows()) = offset.V;
    tri.C.conservativeResize(nf + offset.F.rows(), 3);
    tri.C.bottomRows(offset.F.rows()) = (offset.F.array() + int(nv)).matrix();
    MeshData out;
    io::append_envelope_block(
        out,
        io::geometry_data(tri.V, tri.C, CellType::Triangle),
        CellType::Triangle,
        tri.sets,
        tri.names);
    const Block curves = block(CellType::Edge);
    if (curves.C.rows() > 0) {
        io::append_envelope_block(
            out,
            io::geometry_data(curves.V, curves.C, CellType::Edge),
            CellType::Edge,
            curves.sets,
            curves.names);
    }
    const Block points = block(CellType::Point);
    std::vector<Vector3d> P(size_t(points.V.rows()));
    for (size_t i = 0; i < P.size(); ++i) P[i] = points.V.row(Eigen::Index(i)).transpose();
    io::append_envelope_points(out, P);
    return out;
}

} // namespace wmtk::components::topological_offset
