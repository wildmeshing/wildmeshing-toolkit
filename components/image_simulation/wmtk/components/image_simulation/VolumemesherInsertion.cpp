#include <igl/predicates/ear_clipping.h>
#include <fstream>
#include <set>
#include "ImageSimulationMesh.h"

namespace wmtk::components::image_simulation {

void ImageSimulationMesh::output_embedded_polygon_mesh(
    std::string output_dir,
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::vector<size_t>>& polygon_faces,
    const std::vector<std::vector<size_t>>& polygon_cells,
    const std::vector<bool>& polygon_faces_on_input_surface)
{
    assert(polygon_faces.size() == polygon_faces_on_input_surface.size());

    std::ofstream output(output_dir);
    output.precision(15);
    for (size_t i = 0; i < v_rational.size(); i++) {
        output << "v ";
        for (int j = 0; j < 3; j++) {
            output << v_rational[i][j].get_num_str() << " " << v_rational[i][j].get_den_str()
                   << " ";
        }
        output << std::endl;
    }


    for (size_t i = 0; i < polygon_faces.size(); i++) {
        output << "f ";
        for (int j = 0; j < polygon_faces[i].size(); j++) {
            output << polygon_faces[i][j] << " ";
        }
        output << std::endl;
    }

    for (size_t i = 0; i < polygon_faces_on_input_surface.size(); i++) {
        output << "s " << polygon_faces_on_input_surface[i] << std::endl;
    }

    for (size_t i = 0; i < polygon_cells.size(); i++) {
        output << "c ";
        for (int j = 0; j < polygon_cells[i].size(); j++) {
            output << polygon_cells[i][j] << " ";
        }
        output << std::endl;
    }

    output.close();
}

void ImageSimulationMesh::output_embedded_polygon_surface_mesh(
    std::string output_dir,
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::vector<size_t>>& polygon_faces,
    const std::vector<bool>& polygon_faces_on_input_surface)
{
    assert(polygon_faces.size() == polygon_faces_on_input_surface.size());

    std::ofstream output(output_dir);
    output.precision(15);
    for (size_t i = 0; i < v_rational.size(); i++) {
        output << "v ";
        for (int j = 0; j < 3; j++) {
            output << v_rational[i][j].to_double() << " ";
        }
        output << std::endl;
    }


    for (size_t i = 0; i < polygon_faces.size(); i++) {
        if (!polygon_faces_on_input_surface[i]) continue;
        output << "f ";
        for (int j = 0; j < polygon_faces[i].size(); j++) {
            output << polygon_faces[i][j] + 1 << " ";
        }
        output << std::endl;
    }

    output.close();
}


void ImageSimulationMesh::output_tetrahedralized_embedded_mesh(
    std::string output_dir,
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::array<size_t, 3>>& facets,
    const std::vector<std::array<size_t, 4>>& tets,
    const std::vector<bool>& tet_face_on_input_surface)
{
    assert(tets.size() * 4 == tet_face_on_input_surface.size());

    std::ofstream output(output_dir);
    output.precision(15);

    for (size_t i = 0; i < v_rational.size(); i++) {
        output << "v ";
        for (int j = 0; j < 3; j++) {
            output << v_rational[i][j].get_num_str() << " " << v_rational[i][j].get_den_str()
                   << " ";
        }
        output << std::endl;
    }

    for (size_t i = 0; i < facets.size(); i++) {
        output << "f ";
        for (int j = 0; j < 3; j++) {
            output << facets[i][j] << " ";
        }
        output << std::endl;
    }


    for (size_t i = 0; i < tets.size(); i++) {
        output << "t ";
        for (int j = 0; j < 4; j++) {
            output << tets[i][j] << " ";
        }
        output << std::endl;
    }

    for (size_t i = 0; i < tet_face_on_input_surface.size(); i++) {
        output << "s ";
        for (int j = 0; j < 4; j++) {
            output << tet_face_on_input_surface[i] << " ";
        }
        output << std::endl;
    }

    output.close();
}

void ImageSimulationMesh::output_init_tetmesh(std::string output_dir)
{
    consolidate_mesh();
    std::ofstream output(output_dir);
    output.precision(15);

    auto vs = get_vertices();
    auto fs = get_faces();
    auto ts = get_tets();

    output << vs.size() << std::endl;
    output << fs.size() << std::endl;
    output << ts.size() << std::endl;

    for (auto v : vs) {
        auto vid = v.vid(*this);
        output << "v " << m_vertex_attribute[vid].m_pos[0].get_num_str() << " "
               << m_vertex_attribute[vid].m_pos[0].get_den_str() << " "
               << m_vertex_attribute[vid].m_pos[1].get_num_str() << " "
               << m_vertex_attribute[vid].m_pos[1].get_den_str() << " "
               << m_vertex_attribute[vid].m_pos[2].get_num_str() << " "
               << m_vertex_attribute[vid].m_pos[2].get_den_str() << " "
               << m_vertex_attribute[vid].m_is_on_surface << std::endl;
    }

    for (auto f : fs) {
        auto fid = f.fid(*this);
        auto v1 = f.vid(*this);
        auto v2 = f.switch_vertex(*this).vid(*this);
        auto v3 = f.switch_edge(*this).switch_vertex(*this).vid(*this);

        output << fid << " " << v1 << " " << v2 << " " << v3 << " "
               << m_face_attribute[fid].m_is_surface_fs << " " << m_face_attribute[fid].m_is_bbox_fs
               << std::endl;
    }

    for (auto t : ts) {
        auto tid = t.tid(*this);
        auto vids = oriented_tet_vids(t);

        output << "t " << vids[0] << " " << vids[1] << " " << vids[2] << " " << vids[3]
               << std::endl;
    }

    output.close();
}


bool ImageSimulationMesh::check_polygon_face_validity(std::vector<Vector3r> points)
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
    // output_mesh("background_tetmesh.msh");
    // std::cout << "background_tetmesh written!" << std::endl;

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

    // std::cout << "polygon face on input size: " << embedded_facets_on_input.size() << std::endl;

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
    // for (int i = 0; i < polygon_faces.size(); i++) {
    //     polygon_faces_on_input_surface[i] = false;
    // }
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
                // std::sort(tetra.begin(), tetra.end());
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

bool ImageSimulationMesh::check_nondegenerate_tets()
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

        if (a.dot(b.cross(c)) > 0)
            continue;
        else
            return false;
    }
    return true;
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

    wmtk::logger().info("cnt_round {}/{}", cnt_round, vertices.size());

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

int ImageSimulationMesh::orient3D(
    vol_rem::bigrational px,
    vol_rem::bigrational py,
    vol_rem::bigrational pz,
    vol_rem::bigrational qx,
    vol_rem::bigrational qy,
    vol_rem::bigrational qz,
    vol_rem::bigrational rx,
    vol_rem::bigrational ry,
    vol_rem::bigrational rz,
    vol_rem::bigrational sx,
    vol_rem::bigrational sy,
    vol_rem::bigrational sz)
{
    vol_rem::bigrational fadx, fbdx, fcdx, fady, fbdy, fcdy, fadz, fbdz, fcdz, eb;
    vol_rem::bigrational fbdxcdy, fcdxbdy, fcdxady, fadxcdy, fadxbdy, fbdxady, det;
    fadx = qx - px;
    fbdx = rx - px;
    fcdx = sx - px;
    fady = qy - py;
    fbdy = ry - py;
    fcdy = sy - py;
    fadz = qz - pz;
    fbdz = rz - pz;
    fcdz = sz - pz;
    fbdxcdy = fbdx * fcdy * fadz;
    fcdxbdy = fcdx * fbdy * fadz;
    fcdxady = fcdx * fady * fbdz;
    fadxcdy = fadx * fcdy * fbdz;
    fadxbdy = fadx * fbdy * fcdz;
    fbdxady = fbdx * fady * fcdz;
    det = (fbdxcdy - fcdxbdy) + (fcdxady - fadxcdy) + (fadxbdy - fbdxady);
    return sgn(det);
}

// After having called the following:
// embed_tri_in_poly_mesh(
//    tri_vrt_coords,
//    triangle_indexes,
//    tet_vrt_coords,
//    tet_indexes,
//    vertices,
//    facets,
//    cells,
//    facets_on_input,
//    verbose
// );
//
// the tracked surface can be verified by calling:
//
// if (!checkTrackedFaces(vertices, tri_vrt_coords, facets, facets_on_input, triangle_indexes)) {
//  ... at least a facet in 'facets_on_input' is not coplanar with any facet in 'triangle_indexes'
// }
//
bool ImageSimulationMesh::checkTrackedFaces(
    std::vector<vol_rem::bigrational>& vol_coords,
    const std::vector<double>& surf_coords,
    std::vector<uint32_t>& facets,
    std::vector<uint32_t>& facets_on_input,
    const std::vector<uint32_t>& surf_tris)
{
    std::vector<uint32_t> fstart; // Vector containing the starting index of each face in 'facets'
    for (uint32_t f_i = 0; f_i < facets.size(); f_i += (facets[f_i] + 1)) fstart.push_back(f_i);
    vol_rem::bigrational v1[3], v2[3], v3[3], v4[3];
    for (uint32_t ti : facets_on_input) // For each facet in the tracked surface
    {
        uint32_t start = fstart[ti];
        uint32_t fnv = facets[start]; // num face vertices
        const uint32_t* face_vrts = facets.data() + start + 1;
        size_t i = 0;
        for (; i < surf_tris.size(); i += 3) { // For each input triangular facet 'i'
            v2[0] = surf_coords[surf_tris[i] * 3]; // Let v2,v3,v4 be the coordinates of its three
                                                   // vertices
            v2[1] = surf_coords[surf_tris[i] * 3 + 1];
            v2[2] = surf_coords[surf_tris[i] * 3 + 2];
            v3[0] = surf_coords[surf_tris[i + 1] * 3];
            v3[1] = surf_coords[surf_tris[i + 1] * 3 + 1];
            v3[2] = surf_coords[surf_tris[i + 1] * 3 + 2];
            v4[0] = surf_coords[surf_tris[i + 2] * 3];
            v4[1] = surf_coords[surf_tris[i + 2] * 3 + 1];
            v4[2] = surf_coords[surf_tris[i + 2] * 3 + 2];
            uint32_t v = 0;
            for (; v < fnv; v++) { // For each vertex 'v' of 'ti'
                const uint32_t vid = face_vrts[v];
                v1[0] = vol_coords[vid * 3]; // Let v1 be v's coordinates
                v1[1] = vol_coords[vid * 3 + 1];
                v1[2] = vol_coords[vid * 3 + 2];
                if (orient3D(
                        v1[0],
                        v1[1],
                        v1[2],
                        v2[0],
                        v2[1],
                        v2[2],
                        v3[0],
                        v3[1],
                        v3[2],
                        v4[0],
                        v4[1],
                        v4[2]) != 0)
                    break; // 'v' is not coplanar with triangular facet 'i'
            }
            if (v == fnv) break; // All vertices of 'ti' are coplanar with triangular facet 'i'
        }
        if (i == surf_tris.size())
            return false; // 'ti' is not coplanar with any triangular facet in surf_tris
    }
    return true;
}

int ImageSimulationMesh::orient3D_wmtk_rational(
    wmtk::Rational px,
    wmtk::Rational py,
    wmtk::Rational pz,
    wmtk::Rational qx,
    wmtk::Rational qy,
    wmtk::Rational qz,
    wmtk::Rational rx,
    wmtk::Rational ry,
    wmtk::Rational rz,
    wmtk::Rational sx,
    wmtk::Rational sy,
    wmtk::Rational sz)
{
    wmtk::Rational fadx, fbdx, fcdx, fady, fbdy, fcdy, fadz, fbdz, fcdz, eb;
    wmtk::Rational fbdxcdy, fcdxbdy, fcdxady, fadxcdy, fadxbdy, fbdxady, det;
    fadx = qx - px;
    fbdx = rx - px;
    fcdx = sx - px;
    fady = qy - py;
    fbdy = ry - py;
    fcdy = sy - py;
    fadz = qz - pz;
    fbdz = rz - pz;
    fcdz = sz - pz;
    fbdxcdy = fbdx * fcdy * fadz;
    fcdxbdy = fcdx * fbdy * fadz;
    fcdxady = fcdx * fady * fbdz;
    fadxcdy = fadx * fcdy * fbdz;
    fadxbdy = fadx * fbdy * fcdz;
    fbdxady = fbdx * fady * fcdz;
    det = (fbdxcdy - fcdxbdy) + (fcdxady - fadxcdy) + (fadxbdy - fbdxady);
    if (det > 0)
        return 1;
    else if (det == 0)
        return 0;
    else
        return -1;
}

// After having called the following:
// embed_tri_in_poly_mesh(
//    tri_vrt_coords,
//    triangle_indexes,
//    tet_vrt_coords,
//    tet_indexes,
//    vertices,
//    facets,
//    cells,
//    facets_on_input,
//    verbose
// );
//
// the tracked surface can be verified by calling:
//
// if (!checkTrackedFaces(vertices, tri_vrt_coords, facets, facets_on_input, triangle_indexes)) {
//  ... at least a facet in 'facets_on_input' is not coplanar with any facet in 'triangle_indexes'
// }
//
bool ImageSimulationMesh::checkTrackedFaces_wmtk_rational(
    std::vector<wmtk::Rational>& vol_coords,
    const std::vector<double>& surf_coords,
    std::vector<uint32_t>& facets,
    std::vector<uint32_t>& facets_on_input,
    const std::vector<uint32_t>& surf_tris)
{
    std::vector<uint32_t> fstart; // Vector containing the starting index of each face in 'facets'
    for (uint32_t f_i = 0; f_i < facets.size(); f_i += (facets[f_i] + 1)) fstart.push_back(f_i);
    wmtk::Rational v1[3], v2[3], v3[3], v4[3];
    for (uint32_t ti : facets_on_input) // For each facet in the tracked surface
    {
        uint32_t start = fstart[ti];
        uint32_t fnv = facets[start]; // num face vertices
        const uint32_t* face_vrts = facets.data() + start + 1;
        size_t i = 0;
        for (; i < surf_tris.size(); i += 3) { // For each input triangular facet 'i'
            v2[0] = surf_coords[surf_tris[i] * 3]; // Let v2,v3,v4 be the coordinates of its three
                                                   // vertices
            v2[1] = surf_coords[surf_tris[i] * 3 + 1];
            v2[2] = surf_coords[surf_tris[i] * 3 + 2];
            v3[0] = surf_coords[surf_tris[i + 1] * 3];
            v3[1] = surf_coords[surf_tris[i + 1] * 3 + 1];
            v3[2] = surf_coords[surf_tris[i + 1] * 3 + 2];
            v4[0] = surf_coords[surf_tris[i + 2] * 3];
            v4[1] = surf_coords[surf_tris[i + 2] * 3 + 1];
            v4[2] = surf_coords[surf_tris[i + 2] * 3 + 2];
            uint32_t v = 0;
            for (; v < fnv; v++) { // For each vertex 'v' of 'ti'
                const uint32_t vid = face_vrts[v];
                v1[0] = vol_coords[vid * 3]; // Let v1 be v's coordinates
                v1[1] = vol_coords[vid * 3 + 1];
                v1[2] = vol_coords[vid * 3 + 2];
                if (orient3D_wmtk_rational(
                        v1[0],
                        v1[1],
                        v1[2],
                        v2[0],
                        v2[1],
                        v2[2],
                        v3[0],
                        v3[1],
                        v3[2],
                        v4[0],
                        v4[1],
                        v4[2]) != 0)
                    break; // 'v' is not coplanar with triangular facet 'i'
            }
            if (v == fnv) break; // All vertices of 'ti' are coplanar with triangular facet 'i'
        }
        if (i == surf_tris.size())
            return false; // 'ti' is not coplanar with any triangular facet in surf_tris
    }
    return true;
}

} // namespace wmtk::components::image_simulation