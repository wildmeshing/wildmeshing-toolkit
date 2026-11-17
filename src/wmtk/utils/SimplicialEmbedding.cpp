#include "SimplicialEmbedding.hpp"

#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>

#include <tuple>
#include <vector>

namespace wmtk::utils {

bool cell_keeps_embedding(const TriMesh& m, const TriComplex& K, const TriMesh::Tuple& face)
{
    if (K.face(face)) {
        return true;
    }
    const size_t fid = face.fid(m);
    int n_in = 0;
    for (const size_t v : m.oriented_tri_vids(fid)) {
        n_in += K.vertex(v);
    }
    if (n_in <= 1) {
        return true;
    }
    if (n_in == 3) {
        return false;
    }
    // two: the edge between them must be in K
    for (size_t j = 0; j < 3; ++j) {
        const TriMesh::Tuple e = m.tuple_from_edge(fid, j);
        if (K.vertex(e.vid(m)) && K.vertex(e.switch_vertex(m).vid(m))) {
            return K.edge(e);
        }
    }
    return true;
}

bool cell_keeps_embedding(const TetMesh& m, const TetComplex& K, const TetMesh::Tuple& tet)
{
    if (K.tet(tet)) {
        return true;
    }
    std::vector<size_t> in;
    for (const size_t v : m.oriented_tet_vids(tet)) {
        if (K.vertex(v)) {
            in.push_back(v);
        }
    }
    if (in.size() <= 1) {
        return true;
    }
    if (in.size() == 2) {
        return K.edge(m.tuple_from_edge({{in[0], in[1]}}));
    }
    if (in.size() == 3) {
        return K.face(std::get<0>(m.tuple_from_face({{in[0], in[1], in[2]}})));
    }
    return false;
}

size_t count_embedding_violations(const TriMesh& m, const TriComplex& K)
{
    size_t bad = 0;
    for (const TriMesh::Tuple& f : m.get_faces()) {
        bad += !cell_keeps_embedding(m, K, f);
    }
    return bad;
}

size_t count_embedding_violations(const TetMesh& m, const TetComplex& K)
{
    size_t bad = 0;
    for (const TetMesh::Tuple& t : m.get_tets()) {
        bad += !cell_keeps_embedding(m, K, t);
    }
    return bad;
}

SimplicialEmbeddingStats simplicial_embedding(TriMesh& m, const TriComplex& K)
{
    using Tuple = TriMesh::Tuple;
    SimplicialEmbeddingStats stats;

    // faces outside K whose edges all lie in K
    std::vector<simplex::Face> faces;
    for (const Tuple& f : m.get_faces()) {
        if (K.face(f)) {
            continue;
        }
        const size_t fid = f.fid(m);
        bool all_in = true;
        for (size_t j = 0; j < 3 && all_in; ++j) {
            all_in = K.edge(m.tuple_from_edge(fid, j));
        }
        if (all_in) {
            const auto vs = m.oriented_tri_vids(fid);
            faces.emplace_back(vs[0], vs[1], vs[2]);
        }
    }
    for (const simplex::Face& f : faces) {
        const auto& vs = f.vertices();
        const Tuple t = m.tuple_from_vids(vs[0], vs[1], vs[2]);
        std::vector<Tuple> new_t;
        m.reserve_free_slots(m.cell_slot_bound(t), 1); // serial: grow rather than refuse
        if (m.split_face(t, new_t)) {
            ++stats.faces_split;
        } else {
            ++stats.refused;
        }
    }

    // edges outside K whose two vertices lie in K
    std::vector<simplex::Edge> edges;
    for (const Tuple& e : m.get_edges()) {
        if (K.edge(e)) {
            continue;
        }
        const size_t v0 = e.vid(m);
        const size_t v1 = e.switch_vertex(m).vid(m);
        if (K.vertex(v0) && K.vertex(v1)) {
            edges.emplace_back(v0, v1);
        }
    }
    for (const simplex::Edge& e : edges) {
        // the edge's tuple in its incident face with the smallest vertices
        const auto& v = e.vertices();
        const auto incident = m.simplex_incident_triangles(e).faces();
        const size_t fid = m.tuple_from_simplex(incident.front()).fid(m);
        const Tuple t = m.tuple_from_edge(v[0], v[1], fid);
        std::vector<Tuple> new_t;
        m.reserve_free_slots(m.cell_slot_bound(t), 1);
        if (m.split_edge(t, new_t)) {
            ++stats.edges_split;
        } else {
            ++stats.refused;
        }
    }
    return stats;
}

SimplicialEmbeddingStats simplicial_embedding(TetMesh& m, const TetComplex& K)
{
    using Tuple = TetMesh::Tuple;
    SimplicialEmbeddingStats stats;

    // tets outside K whose faces all lie in K
    std::vector<simplex::Tet> tets;
    for (const Tuple& tet : m.get_tets()) {
        if (K.tet(tet)) {
            continue;
        }
        const auto vs = m.oriented_tet_vids(tet);
        bool all_in = true;
        for (int i = 0; i < 4 && all_in; ++i) {
            all_in =
                K.face(std::get<0>(m.tuple_from_face({{vs[i], vs[(i + 1) % 4], vs[(i + 2) % 4]}})));
        }
        if (all_in) {
            tets.emplace_back(vs[0], vs[1], vs[2], vs[3]);
        }
    }
    for (const simplex::Tet& tet : tets) {
        const auto& vs = tet.vertices();
        const Tuple t = m.tuple_from_vids(vs[0], vs[1], vs[2], vs[3]);
        std::vector<Tuple> new_t;
        m.reserve_free_slots(m.cell_slot_bound(t), 1); // serial: grow rather than refuse
        if (m.split_tet(t, new_t)) {
            ++stats.tets_split;
        } else {
            ++stats.refused;
        }
    }

    // faces outside K whose edges all lie in K
    std::vector<simplex::Face> faces;
    for (const Tuple& f : m.get_faces()) {
        if (K.face(f)) {
            continue;
        }
        const auto vs = m.get_face_vids(f);
        bool all_in = true;
        for (int i = 0; i < 3 && all_in; ++i) {
            all_in = K.edge(m.tuple_from_edge({{vs[i], vs[(i + 1) % 3]}}));
        }
        if (all_in) {
            faces.emplace_back(vs[0], vs[1], vs[2]);
        }
    }
    for (const simplex::Face& f : faces) {
        const auto& vs = f.vertices();
        const Tuple t = std::get<0>(m.tuple_from_face({{vs[0], vs[1], vs[2]}}));
        std::vector<Tuple> new_t;
        m.reserve_free_slots(m.cell_slot_bound(t), 1);
        if (m.split_face(t, new_t)) {
            ++stats.faces_split;
        } else {
            ++stats.refused;
        }
    }

    // edges outside K whose two vertices lie in K
    std::vector<simplex::Edge> edges;
    for (const Tuple& e : m.get_edges()) {
        if (K.edge(e)) {
            continue;
        }
        const size_t v0 = e.vid(m);
        const size_t v1 = m.switch_vertex(e).vid(m);
        if (K.vertex(v0) && K.vertex(v1)) {
            edges.emplace_back(v0, v1);
        }
    }
    for (const simplex::Edge& e : edges) {
        const Tuple t = m.tuple_from_edge(e.vertices());
        std::vector<Tuple> new_t;
        m.reserve_free_slots(m.cell_slot_bound(t), 1);
        if (m.split_edge(t, new_t)) {
            ++stats.edges_split;
        } else {
            ++stats.refused;
        }
    }
    return stats;
}

} // namespace wmtk::utils
