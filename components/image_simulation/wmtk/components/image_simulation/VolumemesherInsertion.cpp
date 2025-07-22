#include <igl/predicates/ear_clipping.h>
#include <fstream>
#include <set>
#include "ImageSimulationMesh.h"

namespace wmtk::components::image_simulation {

std::vector<std::array<size_t, 3>> ImageSimulationMesh::triangulate_polygon_face(
    std::vector<Vector3r> points)
{
    // triangulate weak convex polygons
    std::vector<std::array<size_t, 3>> triangulated_faces;

    std::vector<std::pair<Vector3r, int>> points_vector;
    for (int i = 0; i < points.size(); i++) {
        points_vector.push_back(std::pair<Vector3r, int>(points[i], i));
    }

    // find the first colinear ABC with nonlinear BCD and delete C from vector
    while (points_vector.size() > 3) {
        bool no_colinear = true;
        for (int i = 0; i < points_vector.size(); i++) {
            auto cur = points_vector[i];
            auto next = points_vector[(i + 1) % points_vector.size()];
            auto prev = points_vector[(i + points_vector.size() - 1) % points_vector.size()];
            auto nextnext = points_vector[(i + 2) % points_vector.size()];

            Vector3r a = cur.first - prev.first;
            Vector3r b = next.first - cur.first;
            Vector3r c = nextnext.first - next.first;

            if (((a[0] * b[1] - a[1] * b[0]) == 0 && (a[1] * b[2] - a[2] * b[1]) == 0 &&
                 (a[0] * b[2] - a[2] * b[0]) == 0) &&
                (((b[0] * c[1] - b[1] * c[0]) != 0 || (b[1] * c[2] - b[2] * c[1]) != 0 ||
                  (b[0] * c[2] - b[2] * c[0]) != 0))) {
                no_colinear = false;
                std::array<size_t, 3> t = {
                    size_t(cur.second),
                    size_t(next.second),
                    size_t(nextnext.second)};
                triangulated_faces.push_back(t);
                points_vector.erase(points_vector.begin() + ((i + 1) % points_vector.size()));
                break;
            } else {
                continue;
            }
        }

        if (no_colinear) break;
    }

    // cleanup convex polygon
    while (points_vector.size() >= 3) {
        std::array<size_t, 3> t = {
            size_t(points_vector[0].second),
            size_t(points_vector[1].second),
            size_t(points_vector[points_vector.size() - 1].second)};
        triangulated_faces.push_back(t);
        points_vector.erase(points_vector.begin());
    }

    return triangulated_faces;
}

// we have the vertices and triangles
// to generate what we need for volumemesher
// coords, ncoords, tri_idx, ntri_idx

// embed input surface on generated back ground mesh

void ImageSimulationMesh::insertion_by_volumeremesher(
    const std::vector<Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    std::vector<Vector3r>& v_rational,
    std::vector<std::array<size_t, 3>>& facets_after,
    std::vector<bool>& is_v_on_input,
    std::vector<std::array<size_t, 4>>& tets_after,
    std::vector<bool>& tet_face_on_input_surface)
{
    std::cout << "vertices size: " << vertices.size() << std::endl;
    std::cout << "faces size: " << faces.size() << std::endl;

    // generate background mesh
    init_from_delaunay_box_mesh(vertices);

    // prepare tet vertices and tet index info

    auto tet_vers = get_vertices();
    auto tets = get_tets();
    std::vector<double> tet_ver_coord(3 * tet_vers.size());
    std::vector<uint32_t> tet_index(4 * tets.size());
    std::cout << "tetver size: " << tet_vers.size() << std::endl;
    std::cout << "tet size: " << tets.size() << std::endl;

    for (int i = 0; i < tet_vers.size(); ++i) {
        tet_ver_coord[3 * i] = m_vertex_attribute[i].m_posf[0];
        tet_ver_coord[3 * i + 1] = m_vertex_attribute[i].m_posf[1];
        tet_ver_coord[3 * i + 2] = m_vertex_attribute[i].m_posf[2];
    }

    for (int i = 0; i < tets.size(); ++i) {
        auto tet_vids = oriented_tet_vids(tets[i]);
        tet_index[4 * i] = tet_vids[0];
        tet_index[4 * i + 1] = tet_vids[1];
        tet_index[4 * i + 2] = tet_vids[2];
        tet_index[4 * i + 3] = tet_vids[3];
    }

    // prepare input surfaces info
    std::vector<double> tri_ver_coord(3 * vertices.size());
    std::vector<uint32_t> tri_index(3 * faces.size());

    for (int i = 0; i < vertices.size(); ++i) {
        tri_ver_coord[3 * i] = vertices[i][0];
        tri_ver_coord[3 * i + 1] = vertices[i][1];
        tri_ver_coord[3 * i + 2] = vertices[i][2];
    }

    for (int i = 0; i < faces.size(); ++i) {
        tri_index[3 * i] = faces[i][0];
        tri_index[3 * i + 1] = faces[i][1];
        tri_index[3 * i + 2] = faces[i][2];
    }

    std::cout << tri_ver_coord.size() << std::endl;
    std::cout << tri_index.size() << std::endl;
    std::cout << tet_ver_coord.size() << std::endl;
    std::cout << tet_index.size() << std::endl;

    std::vector<vol_rem::bigrational> embedded_vertices;
    std::vector<uint32_t> embedded_facets;
    std::vector<uint32_t> embedded_cells;
    std::vector<uint32_t> embedded_facets_on_input;

    std::vector<std::array<uint32_t, 4>> out_tets;
    std::vector<uint32_t> final_tets_parent;
    std::vector<bool> cells_with_faces_on_input;
    std::vector<std::vector<uint32_t>> final_tets_parent_faces;

    // volumeremesher embed
    vol_rem::embed_tri_in_poly_mesh(
        tri_ver_coord,
        tri_index,
        tet_ver_coord,
        tet_index,
        embedded_vertices,
        embedded_facets,
        embedded_cells,
        out_tets,
        final_tets_parent,
        embedded_facets_on_input,
        cells_with_faces_on_input,
        final_tets_parent_faces,
        true);

    //v_rational.resize(embedded_vertices.size() / 3);
    for (int i = 0; i < embedded_vertices.size() / 3; i++) {
        v_rational.push_back(Vector3r());
        v_rational.back()[0].init(embedded_vertices[3 * i].get_mpq_t());
        v_rational.back()[1].init(embedded_vertices[3 * i + 1].get_mpq_t());
        v_rational.back()[2].init(embedded_vertices[3 * i + 2].get_mpq_t());
    }

    std::vector<std::vector<size_t>> polygon_faces;
    int polycnt = 0;
    for (int i = 0; i < embedded_facets.size(); i++) {
        int polysize = embedded_facets[i];
        std::vector<size_t> polygon;
        for (int j = i + 1; j <= i + polysize; j++) {
            polygon.push_back(embedded_facets[j]);
        }
        polycnt++;
        polygon_faces.push_back(polygon);
        i += polysize;
    }

    std::vector<std::vector<size_t>> polygon_cells;
    std::vector<std::array<size_t, 4>> tets_final;
    for (int i = 0; i < embedded_cells.size(); i++) {
        std::vector<size_t> polygon_cell;
        int cellsize = embedded_cells[i];
        for (int j = i + 1; j <= i + cellsize; j++) {
            polygon_cell.push_back(embedded_cells[j]);
        }
        polygon_cells.push_back(polygon_cell);
        i += cellsize;
    }

    std::cout << "polygon cells num: " << polygon_cells.size() << std::endl;

    std::vector<bool> polygon_faces_on_input_surface(polygon_faces.size(), false);

    for (int i = 0; i < embedded_facets_on_input.size(); i++) {
        polygon_faces_on_input_surface[embedded_facets_on_input[i]] = true;
    }

    std::vector<std::array<size_t, 3>> triangulated_faces;
    std::vector<bool> triangulated_faces_on_input;
    std::vector<std::vector<size_t>> map_poly_to_tri_face(polygon_faces.size());

    int poly_cnt = 0;

    // triangulate polygon faces
    for (int i = 0; i < polygon_faces.size(); i++) {
        // already clipped in other polygon
        if (map_poly_to_tri_face[i].size() != 0) continue;

        // new polygon face to clip
        std::vector<std::array<size_t, 3>> clipped_indices;
        std::vector<Vector3r> poly_coordinates;
        std::vector<size_t> polygon_face = polygon_faces[i];
        assert(polygon_face.size() >= 3);

        if (polygon_face.size() == 3) {
            // already a triangle
            std::array<size_t, 3> triangle_face = {
                polygon_face[0],
                polygon_face[1],
                polygon_face[2]};
            int idx = triangulated_faces.size();
            triangulated_faces.push_back(triangle_face);
            if (polygon_faces_on_input_surface[i]) {
                triangulated_faces_on_input.push_back(true);
            } else {
                triangulated_faces_on_input.push_back(false);
            }
            map_poly_to_tri_face[i].push_back(idx);
        } else {
            poly_cnt++;
            for (int j = 0; j < polygon_faces[i].size(); j++) {
                poly_coordinates.push_back(v_rational[polygon_face[j]]);
            }

            clipped_indices = triangulate_polygon_face(poly_coordinates);
            for (int j = 0; j < clipped_indices.size(); j++) {
                // need to map oldface index to new face indices
                std::array<size_t, 3> triangle_face = {
                    polygon_face[clipped_indices[j][0]],
                    polygon_face[clipped_indices[j][1]],
                    polygon_face[clipped_indices[j][2]]};
                int idx = triangulated_faces.size();
                triangulated_faces.push_back(triangle_face);

                // track input faces
                if (polygon_faces_on_input_surface[i]) {
                    triangulated_faces_on_input.push_back(true);
                } else {
                    triangulated_faces_on_input.push_back(false);
                }
                map_poly_to_tri_face[i].push_back(idx);
            }
        }
    }

    std::cout << "poly_cnt:" << poly_cnt << std::endl;
    std::cout << "finish triangulation" << std::endl;
    std::cout << "vertice before tetra num: " << v_rational.size() << std::endl;

    int was_tet_cnt = 0;
    for (int i = 0; i < polygon_cells.size(); i++) {
        auto polygon_cell = polygon_cells[i];

        // get polygon vertices
        std::vector<size_t> polygon_vertices;
        for (auto f : polygon_cell) {
            for (auto v : polygon_faces[f]) {
                polygon_vertices.push_back(v);
            }
        }
        wmtk::vector_unique(polygon_vertices);

        // compute number of triangle faces
        int num_faces = 0;
        for (auto f : polygon_cell) {
            num_faces += map_poly_to_tri_face[f].size();
        }

        // polygon already a tet
        if (num_faces == 4) {
            was_tet_cnt++;
            assert(polygon_vertices.size() == 4);
            // get the correct orientation here
            size_t v0 = polygon_faces[polygon_cell[0]][0];
            size_t v1 = polygon_faces[polygon_cell[0]][1];
            size_t v2 = polygon_faces[polygon_cell[0]][2];
            size_t v3;
            for (auto v : polygon_faces[polygon_cell[1]]) {
                if (v != v0 && v != v1 && v != v2) {
                    v3 = v;
                    break;
                }
            }

            std::array<size_t, 4> tetra = {v0, v1, v2, v3};

            // if inverted then fix the orientation
            Vector3r v0v1 = v_rational[v1] - v_rational[v0];
            Vector3r v0v2 = v_rational[v2] - v_rational[v0];
            Vector3r v0v3 = v_rational[v3] - v_rational[v0];
            if ((v0v1.cross(v0v2)).dot(v0v3) < 0) {
                tetra = {v1, v0, v2, v3};
            }

            // push the tet to final queue;
            tets_final.push_back(tetra);

            std::set<size_t> local_f1 = {tetra[0], tetra[1], tetra[2]};
            std::set<size_t> local_f2 = {tetra[0], tetra[2], tetra[3]};
            std::set<size_t> local_f3 = {tetra[0], tetra[1], tetra[3]};
            std::set<size_t> local_f4 = {tetra[1], tetra[2], tetra[3]};

            // track surface     need to be fixed
            bool tet_face_on_input[4];
            for (auto f : polygon_cell) {
                std::set<size_t> f_vs = {
                    polygon_faces[f][0],
                    polygon_faces[f][1],
                    polygon_faces[f][2]};

                int local_f_idx;

                // decide which face it is

                if (f_vs == local_f1) {
                    local_f_idx = 0;
                } else if (f_vs == local_f2) {
                    local_f_idx = 1;
                } else if (f_vs == local_f3) {
                    local_f_idx = 2;
                } else {
                    local_f_idx = 3;
                }

                tet_face_on_input[local_f_idx] = polygon_faces_on_input_surface[f];
            }

            for (int k = 0; k < 4; k++) {
                tet_face_on_input_surface.push_back(tet_face_on_input[k]);
            }
            continue;
        }

        // compute centroid
        Vector3r centroid(0, 0, 0);
        for (auto v : polygon_vertices) {
            centroid = centroid + v_rational[v];
        }
        centroid = centroid / polygon_vertices.size();

        // trahedralize
        size_t centroid_idx = v_rational.size();
        v_rational.push_back(centroid);

        for (auto f : polygon_cell) {
            for (auto t : map_poly_to_tri_face[f]) {
                std::array<size_t, 4> tetra = {
                    triangulated_faces[t][0],
                    triangulated_faces[t][1],
                    triangulated_faces[t][2],
                    centroid_idx};
                // check inverted tet and fix
                Vector3r v0v1 = v_rational[tetra[1]] - v_rational[tetra[0]];
                Vector3r v0v2 = v_rational[tetra[2]] - v_rational[tetra[0]];
                Vector3r v0v3 = v_rational[tetra[3]] - v_rational[tetra[0]];
                if ((v0v1.cross(v0v2)).dot(v0v3) < 0) {
                    tetra = {
                        triangulated_faces[t][1],
                        triangulated_faces[t][0],
                        triangulated_faces[t][2],
                        centroid_idx};
                }

                tets_final.push_back(tetra);
                tet_face_on_input_surface.push_back(triangulated_faces_on_input[t]);
                tet_face_on_input_surface.push_back(false);
                tet_face_on_input_surface.push_back(false);
                tet_face_on_input_surface.push_back(false);
            }
        }
    }

    std::cout << "polygon was tet num: " << was_tet_cnt << std::endl;
    std::cout << "vertices final num: " << v_rational.size() << std::endl;
    std::cout << "tets final num: " << tets_final.size() << std::endl;

    std::cout << "track face size: " << tet_face_on_input_surface.size() << std::endl;

    facets_after = triangulated_faces;
    tets_after = tets_final;

    // track vertices on input
    is_v_on_input.reserve(v_rational.size());
    for (int i = 0; i < v_rational.size(); i++) {
        is_v_on_input.push_back(false);
    }
    for (int i = 0; i < triangulated_faces.size(); i++) {
        if (triangulated_faces_on_input[i]) {
            is_v_on_input[triangulated_faces[i][0]] = true;
            is_v_on_input[triangulated_faces[i][1]] = true;
            is_v_on_input[triangulated_faces[i][2]] = true;
        }
    }

    size_t on_surface_v_cnt = 0;
    for (size_t i = 0; i < is_v_on_input.size(); i++) {
        if (is_v_on_input[i]) on_surface_v_cnt++;
    }

    std::cout << "v on surface vector size: " << is_v_on_input.size();
    std::cout << "v on surface: " << on_surface_v_cnt << std::endl;
}

void ImageSimulationMesh::init_from_Volumeremesher(
    const std::vector<Vector3r>& v_rational,
    const std::vector<bool>& is_v_on_input,
    const std::vector<std::array<size_t, 4>>& tets,
    const std::vector<bool>& tet_face_on_input_surface)
{
    assert(tet_face_on_input_surface.size() == 4 * tets.size());

    init_with_isolated_vertices(v_rational.size(), tets);
    assert(check_mesh_connectivity_validity());

    m_vertex_attribute.m_attributes.resize(v_rational.size());
    m_tet_attribute.m_attributes.resize(tets.size());
    m_face_attribute.m_attributes.resize(tets.size() * 4);

    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_pos = v_rational[i];
        m_vertex_attribute[i].m_posf = to_double(v_rational[i]);
    }

    // check here
    for (size_t i = 0; i < tet_face_on_input_surface.size(); i++) {
        if (tet_face_on_input_surface[i]) {
            m_face_attribute[i].m_is_surface_fs = 1;
        }
    }

    const auto faces = get_faces();
    std::cout << "faces size: " << faces.size() << std::endl;
    for (const Tuple& f : faces) {
        SmartTuple ff(*this, f);
        const size_t fid = ff.fid();
        const size_t v1 = ff.vid();
        const size_t v2 = ff.switch_vertex().vid();
        const size_t v3 = ff.switch_edge().switch_vertex().vid();
        if (m_face_attribute[fid].m_is_surface_fs == 1) {
            assert(is_v_on_input[v1] && is_v_on_input[v2] && is_v_on_input[v3]);
            m_vertex_attribute[v1].m_is_on_surface = true;
            m_vertex_attribute[v2].m_is_on_surface = true;
            m_vertex_attribute[v3].m_is_on_surface = true;
        }
    }

    // track bounding box
    for (size_t i = 0; i < faces.size(); i++) {
        const auto vs = get_face_vertices(faces[i]);
        std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
        int on_bbox = -1;
        for (int k = 0; k < 3; k++) {
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_min[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_min[k] &&
                m_vertex_attribute[vids[2]].m_pos[k] == m_params.box_min[k]) {
                on_bbox = k * 2;
                break;
            }
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_max[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_max[k] &&
                m_vertex_attribute[vids[2]].m_pos[k] == m_params.box_max[k]) {
                on_bbox = k * 2 + 1;
                break;
            }
        }
        if (on_bbox < 0) {
            continue;
        }
        assert(!faces[i].switch_tetrahedron(*this)); // face must be on boundary

        const size_t fid = faces[i].fid(*this);
        m_face_attribute[fid].m_is_bbox_fs = on_bbox;

        for (const size_t vid : vids) {
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
        }
    }

    for_each_vertex(
        [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });

    // track open boundaries
    find_open_boundary();

    int open_boundary_cnt = 0;
    for (const Tuple& e : get_edges()) {
        if (is_open_boundary_edge(e)) {
            open_boundary_cnt++;
        }
    }
    wmtk::logger().info("#open boundary edges: {}", open_boundary_cnt);

    // // rounding
    size_t cnt_round = 0;

    const auto vertices = get_vertices();
    for (const Tuple& v : vertices) {
        if (round(v)) {
            cnt_round++;
        }
    }

    if (cnt_round != vertices.size()) {
        log_and_throw_error(
            "Could not round all vertices in tet mesh. Rounded {}/{}",
            cnt_round,
            vertices.size());
    }

    // init qualities
    for_each_tetra(
        [this](const Tuple& t) { m_tet_attribute[t.tid(*this)].m_quality = get_quality(t); });
}

void ImageSimulationMesh::init_from_image(
    const MatrixXd& V,
    const MatrixXi& T,
    const VectorXi& T_tags)
{
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T_tags.size() == T.rows());

    init(T);

    assert(check_mesh_connectivity_validity());

    m_vertex_attribute.m_attributes.resize(V.rows());
    m_tet_attribute.m_attributes.resize(T.rows());
    m_face_attribute.m_attributes.resize(T.rows() * 4);

    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_pos = to_rational(V.row(i));
        m_vertex_attribute[i].m_posf = V.row(i);
    }

    // add tags
    for (size_t i = 0; i < T_tags.size(); ++i) {
        m_tet_attribute[i].tag = T_tags[i];
    }

    const auto faces = get_faces();
    std::cout << "faces size: " << faces.size() << std::endl;

    std::vector<Eigen::Vector3i> tempF;
    for (const Tuple& f : faces) {
        SmartTuple ff(*this, f);

        const auto t_opp = ff.switch_tetrahedron();
        if (!t_opp) {
            continue;
        }
        const int64_t tag0 = m_tet_attribute[ff.tid()].tag;
        const int64_t tag1 = m_tet_attribute[t_opp.value().tid()].tag;

        if (tag0 == tag1) {
            continue;
        }

        m_face_attribute[ff.fid()].m_is_surface_fs = 1;

        const size_t v1 = ff.vid();
        const size_t v2 = ff.switch_vertex().vid();
        const size_t v3 = ff.switch_edge().switch_vertex().vid();
        m_vertex_attribute[v1].m_is_on_surface = true;
        m_vertex_attribute[v2].m_is_on_surface = true;
        m_vertex_attribute[v3].m_is_on_surface = true;

        tempF.emplace_back(v1, v2, v3);
    }

    // build envelopes
    std::vector<Eigen::Vector3d> tempV(V.rows());
    for (size_t i = 0; i < V.rows(); ++i) {
        tempV[i] = V.row(i);
    }
    m_envelope = std::make_shared<ExactEnvelope>();
    m_envelope->init(tempV, tempF, m_envelope_eps);
    triangles_tree = std::make_shared<SampleEnvelope>();
    triangles_tree->init(tempV, tempF, m_envelope_eps);

    // track bounding box
    for (size_t i = 0; i < faces.size(); i++) {
        const auto vs = get_face_vertices(faces[i]);
        std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
        int on_bbox = -1;
        for (int k = 0; k < 3; k++) {
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_min[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_min[k] &&
                m_vertex_attribute[vids[2]].m_pos[k] == m_params.box_min[k]) {
                on_bbox = k * 2;
                break;
            }
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_max[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_max[k] &&
                m_vertex_attribute[vids[2]].m_pos[k] == m_params.box_max[k]) {
                on_bbox = k * 2 + 1;
                break;
            }
        }
        if (on_bbox < 0) {
            continue;
        }
        assert(!faces[i].switch_tetrahedron(*this)); // face must be on boundary

        const size_t fid = faces[i].fid(*this);
        m_face_attribute[fid].m_is_bbox_fs = on_bbox;

        for (const size_t vid : vids) {
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
        }
    }

    for_each_vertex(
        [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });

    // track open boundaries
    find_open_boundary();

    int open_boundary_cnt = 0;
    for (const Tuple& e : get_edges()) {
        if (is_open_boundary_edge(e)) {
            open_boundary_cnt++;
        }
    }
    wmtk::logger().info("#open boundary edges: {}", open_boundary_cnt);

    // // rounding
    size_t cnt_round = 0;

    const auto vertices = get_vertices();
    for (const Tuple& v : vertices) {
        if (round(v)) {
            cnt_round++;
        }
    }

    if (cnt_round != vertices.size()) {
        log_and_throw_error(
            "Could not round all vertices in tet mesh. Rounded {}/{}",
            cnt_round,
            vertices.size());
    }

    // init qualities
    for_each_tetra(
        [this](const Tuple& t) { m_tet_attribute[t.tid(*this)].m_quality = get_quality(t); });
}


void ImageSimulationMesh::find_open_boundary()
{
    const auto faces = get_faces();
    std::vector<int> edge_on_open_boundary(6 * tet_capacity(), 0);

    // for open boundary envelope
    std::vector<Eigen::Vector3d> v_posf(vert_capacity());
    std::vector<Eigen::Vector3i> open_boundaries;

    for (size_t i = 0; i < vert_capacity(); i++) {
        v_posf[i] = m_vertex_attribute[i].m_posf;
    }

    for (const Tuple& f : faces) {
        const SmartTuple ff(*this, f);
        const size_t fid = ff.fid();
        if (!m_face_attribute[fid].m_is_surface_fs) {
            continue;
        }
        size_t eid1 = ff.eid();
        size_t eid2 = ff.switch_edge().eid();
        size_t eid3 = ff.switch_vertex().switch_edge().eid();

        edge_on_open_boundary[eid1]++;
        edge_on_open_boundary[eid2]++;
        edge_on_open_boundary[eid3]++;
    }

    const auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (edge_on_open_boundary[e.eid(*this)] != 1) {
            continue;
        }
        size_t v1 = e.vid(*this);
        size_t v2 = e.switch_vertex(*this).vid(*this);
        m_vertex_attribute[v1].m_is_on_open_boundary = true;
        m_vertex_attribute[v2].m_is_on_open_boundary = true;
        open_boundaries.emplace_back(v1, v2, v1); // degenerate triangle to mimic the edge
    }

    wmtk::logger().info("open boundary num: {}", open_boundaries.size());

    if (open_boundaries.size() == 0) {
        return;
    }

    // init open boundary envelope
    m_open_boundary_envelope.init(v_posf, open_boundaries, m_params.epsr * m_params.diag_l / 2.0);
    boundaries_tree.init(v_posf, open_boundaries, m_params.epsr * m_params.diag_l / 2.0);
}

bool ImageSimulationMesh::is_open_boundary_edge(const Tuple& e)
{
    size_t v1 = e.vid(*this);
    size_t v2 = e.switch_vertex(*this).vid(*this);
    if (!m_vertex_attribute[v1].m_is_on_open_boundary ||
        !m_vertex_attribute[v2].m_is_on_open_boundary)
        return false;

    /*
     * This code is not reliable. If the envelope is chosen too large, elements could be reported as
     * boundary even though they aren't. Especially, when there are sliver triangles that are barely
     * not inverted.
     */

    return !m_open_boundary_envelope.is_outside(
        {{m_vertex_attribute[v1].m_posf,
          m_vertex_attribute[v2].m_posf,
          m_vertex_attribute[v1].m_posf}});
}

} // namespace wmtk::components::image_simulation