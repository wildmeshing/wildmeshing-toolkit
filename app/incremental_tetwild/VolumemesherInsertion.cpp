#include <igl/predicates/ear_clipping.h>
#include "IncrementalTetWild.h"

bool tetwild::TetWild::check_polygon_face_validity(std::vector<tetwild::Vector3r> points)
{
    if (points.size() == 3) return true;
    if (points.size() < 3) return false;
    for (int i = 0; i < points.size() - 3; i++) {
        Eigen::Matrix<wmtk::Rational, 4, 4> tet;
        tet << points[i][0], points[i][1], points[i][2], 1, //
            points[i + 1][0], points[i + 1][1], points[i + 1][2], 1, //
            points[i + 2][0], points[i + 2][1], points[i + 2][2], 1, //
            points[i + 3][0], points[i + 3][1], points[i + 3][2], 1;
        wmtk::Rational tet_volume = tet.determinant();
        if (tet_volume != 0) {
            std::cout << tet_volume.to_double() << std::endl;
            return false;
        }
    }
    return true;
}

std::vector<std::array<size_t, 3>> tetwild::TetWild::triangulate_polygon_face(
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
                std::array<size_t, 3> t = {cur.second, next.second, nextnext.second};
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
            points_vector[0].second,
            points_vector[1].second,
            points_vector[points_vector.size() - 1].second};
        triangulated_faces.push_back(t);
        points_vector.erase(points_vector.begin());
    }

    return triangulated_faces;
}


// we have the vertices and triangles
// to generate what we need for volumemesher
// coords, ncoords, tri_idx, ntri_idx

// embed input surface on generated back ground mesh

void tetwild::TetWild::insertion_by_volumeremesher(
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
    // tet_ver_coord.reserve(3 * tet_vers.size());
    // tet_index.reserve(4 * tets.size());

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
    // tri_ver_coord.reserve(3 * vertices.size());
    // tri_index.reserve(3 * faces.size());

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

    // volumeremesher embed
    vol_rem::embed_tri_in_poly_mesh(
        tri_ver_coord,
        tri_index,
        tet_ver_coord,
        tet_index,
        embedded_vertices,
        embedded_facets,
        embedded_cells,
        embedded_facets_on_input,
        true);

    // v_rational.reserve(embedded_vertices.size()/3);

    std::cout << "polygon face on input size: " << embedded_facets_on_input.size() << std::endl;

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

    // check polygon face validity
    for (int i = 0; i < polygon_faces.size(); i++) {
        std::vector<tetwild::Vector3r> polyface_coordinates;
        for (int j = 0; j < polygon_faces[i].size(); j++) {
            polyface_coordinates.push_back(v_rational[polygon_faces[i][j]]);
        }
        if (!check_polygon_face_validity(polyface_coordinates)) {
            std::cout << "polygon face invalid!!: " << i << std::endl;
        }
    }

    std::vector<bool> polygon_faces_on_input_surface(polygon_faces.size());
    for (int i = 0; i < polygon_faces.size(); i++) {
        polygon_faces_on_input_surface[i] = false;
    }
    for (int i = 0; i < embedded_facets_on_input.size(); i++) {
        polygon_faces_on_input_surface[embedded_facets_on_input[i]] = true;
    }

    std::vector<std::array<size_t, 3>> triangulated_faces;
    std::vector<bool> triangulated_faces_on_input;
    std::vector<std::vector<size_t>> map_poly_to_tri_face(polygon_faces.size());

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
            // std::cout<<std::endl<<"polyface: ";
            // for (int j=0; j<polygon_face.size(); j++){
            // std::cout<<polygon_face[j]<<" ";
            // }
            // std::cout<<std::endl<<"coords: "<<std::endl;
            for (int j = 0; j < polygon_faces[i].size(); j++) {
                poly_coordinates.push_back(v_rational[polygon_face[j]]);
                // std::cout<<v_rational[polygon_face[j]][0]<<" "<<v_rational[polygon_face[j]][1]<<"
                // "<<v_rational[polygon_face[j]][2]<<std::endl;
            }
            // std::cout<<std::endl;

            clipped_indices = triangulate_polygon_face(poly_coordinates);
            // std::cout<<"clipped indices size: "<<clipped_indices.size()<<std::endl;
            for (int j = 0; j < clipped_indices.size(); j++) {
                // need to map oldface index to new face indices
                std::array<size_t, 3> triangle_face = {
                    polygon_face[clipped_indices[j][0]],
                    polygon_face[clipped_indices[j][1]],
                    polygon_face[clipped_indices[j][2]]};
                // std::cout<<triangle_face[0]<<" "<<triangle_face[1]<<"
                // "<<triangle_face[2]<<std::endl;
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

    std::cout << triangulated_faces.size() << std::endl;
    int sum = 0;
    for (int i = 0; i < map_poly_to_tri_face.size(); i++) {
        sum += map_poly_to_tri_face[i].size();
    }
    std::cout << sum << std::endl;

    std::cout << "triangulated faces on input vector size: " << triangulated_faces_on_input.size()
              << std::endl;
    int on_sur_cnt = 0;
    for (int i = 0; i < triangulated_faces_on_input.size(); i++) {
        if (triangulated_faces_on_input[i]) on_sur_cnt++;
    }
    std::cout << "triangulated faces on input: " << on_sur_cnt << std::endl;

    std::cout << "finish triangulation" << std::endl;

    std::cout << "vertice before tetra num: " << v_rational.size() << std::endl;

    // tetrahedralize cells

    // track face on surface per tet
    // std::vector<bool> tet_face_on_input_surface;

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
            std::array<size_t, 4> tetra = {
                polygon_vertices[0],
                polygon_vertices[1],
                polygon_vertices[2],
                polygon_vertices[3]};
            tets_final.push_back(tetra);

            // track surface
            bool tet_face_on_input[4];
            for (auto f : polygon_cell) {
                std::vector<size_t> f_vs(3);
                f_vs[0] = polygon_faces[f][0];
                f_vs[1] = polygon_faces[f][1];
                f_vs[2] = polygon_faces[f][2];
                std::sort(f_vs.begin(), f_vs.end());

                int local_f_idx;

                // decide which face it is

                if (f_vs[0] == polygon_vertices[0] && f_vs[1] == polygon_vertices[1] &&
                    f_vs[2] == polygon_vertices[2]) {
                    //  0 1 2
                    local_f_idx = 0;
                } else if (
                    f_vs[0] == polygon_vertices[0] && f_vs[1] == polygon_vertices[2] &&
                    f_vs[2] == polygon_vertices[3]) {
                    // 0 2 3
                    local_f_idx = 1;
                } else if (
                    f_vs[0] == polygon_vertices[0] && f_vs[1] == polygon_vertices[1] &&
                    f_vs[2] == polygon_vertices[3]) {
                    // 0 1 3
                    local_f_idx = 2;
                } else {
                    // 1 2 3
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
                    centroid_idx,
                    triangulated_faces[t][0],
                    triangulated_faces[t][1],
                    triangulated_faces[t][2]};
                tets_final.push_back(tetra);
                tet_face_on_input_surface.push_back(false);
                tet_face_on_input_surface.push_back(false);
                tet_face_on_input_surface.push_back(false);
                tet_face_on_input_surface.push_back(triangulated_faces_on_input[t]);
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
    for (int i = 0; i < v_rational.size(); i++) is_v_on_input.push_back(false);
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

bool tetwild::TetWild::check_nondegenerate_tets()
{
    auto tets = get_tets();
    // std::cout << tets.size();
    for (int i = 0; i < tets.size(); i++) {
        // std::cout << "-----tet " << i << "-----" << std::endl;
        auto v1 = tets[i].vid(*this);
        auto v2 = tets[i].switch_vertex(*this).vid(*this);
        auto v3 = tets[i].switch_edge(*this).switch_vertex(*this).vid(*this);
        auto v4 = tets[i].switch_face(*this).switch_edge(*this).switch_vertex(*this).vid(*this);

        Vector3r a(m_vertex_attribute[v2].m_pos - m_vertex_attribute[v1].m_pos);
        Vector3r b(m_vertex_attribute[v3].m_pos - m_vertex_attribute[v1].m_pos);
        Vector3r c(m_vertex_attribute[v4].m_pos - m_vertex_attribute[v1].m_pos);

        // std::cout << v1 << " " << v2 << " " << v3 << " " << v4 << std::endl;

        if (a.dot(b.cross(c)) != 0)
            continue;
        else
            return false;
    }
    return true;
}

void tetwild::TetWild::init_from_Volumeremesher(
    std::vector<Vector3r>& v_rational,
    std::vector<std::array<size_t, 3>>& facets,
    std::vector<bool>& is_v_on_input,
    std::vector<std::array<size_t, 4>>& tets,
    std::vector<bool>& tet_face_on_input_surface)
{
    init_with_isolated_vertices(v_rational.size(), tets);
    assert(check_mesh_connectivity_validity());

    m_vertex_attribute.m_attributes.resize(v_rational.size());
    m_tet_attribute.m_attributes.resize(tets.size());
    m_face_attribute.m_attributes.resize(tets.size() * 4);

    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_pos = v_rational[i];
        m_vertex_attribute[i].m_posf = to_double(v_rational[i]);
    }

    // track faces

    for (int i = 0; i < is_v_on_input.size(); i++) {
        m_vertex_attribute[i].m_is_on_surface = is_v_on_input[i];
    }

    auto faces = get_faces();
    std::cout << "faces size: " << faces.size() << std::endl;
    // for (auto f : faces) {
    //     auto v1 = f.vid(*this);
    //     auto v2 = f.switch_vertex(*this).vid(*this);
    //     auto v3 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
    //     if (m_vertex_attribute[v1].m_is_on_surface && m_vertex_attribute[v2].m_is_on_surface
    //     &&
    //         m_vertex_attribute[v3].m_is_on_surface) {
    //         m_face_attribute[f.fid(*this)].m_is_surface_fs = 1;
    //     }
    // }
    for (size_t i = 0; i < tet_face_on_input_surface.size(); i++) {
        if (tet_face_on_input_surface[i]) m_face_attribute[i].m_is_surface_fs = 1;
    }

    // track bounding box

    for (size_t i = 0; i < faces.size(); i++) {
        auto vs = get_face_vertices(faces[i]);
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
        if (on_bbox < 0) continue;
        auto fid = faces[i].fid(*this);
        m_face_attribute[fid].m_is_bbox_fs = on_bbox;

        for (size_t vid : vids) {
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
        }
    }

    // enable on for tests
    if (!check_nondegenerate_tets()) {
        std::cout << "find tet with 0 volume!" << std::endl;
    }

    if (!check_mesh_connectivity_validity()) {
        std::cout << "invalid mesh connectivity!" << std::endl;
    }
}
