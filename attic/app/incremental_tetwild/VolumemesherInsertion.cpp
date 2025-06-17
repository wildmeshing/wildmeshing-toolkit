#include <igl/predicates/ear_clipping.h>
#include <fstream>
#include <set>
#include "IncrementalTetWild.h"

void tetwild::TetWild::output_embedded_polygon_mesh(
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

void tetwild::TetWild::output_embedded_polygon_surface_mesh(
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


void tetwild::TetWild::output_tetrahedralized_embedded_mesh(
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

void tetwild::TetWild::output_init_tetmesh(std::string output_dir)
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
    output_mesh("background_tetmesh.msh");
    std::cout << "background_tetmesh written!" << std::endl;

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

    // marco's test
    // if (!checkTrackedFaces(
    //         embedded_vertices,
    //         tri_ver_coord,
    //         embedded_facets,
    //         embedded_facets_on_input,
    //         tri_index)) {
    //     std::cout
    //         << "WARNING: at least a facet in 'facets_on_input' is not coplanar with any facet in
    //         "
    //            "triangle_indexes with BIGRATIONAL"
    //         << std::endl;
    // }

    // test use
    // std::vector<wmtk::Rational> v_rational_for_test(embedded_vertices.size());
    // for (int i = 0; i < embedded_vertices.size(); i++) {
    //     v_rational_for_test[i].init(embedded_vertices[i].get_mpq_t());
    // }

    // if (!checkTrackedFaces_wmtk_rational(
    //         v_rational_for_test,
    //         tri_ver_coord,
    //         embedded_facets,
    //         embedded_facets_on_input,
    //         tri_index)) {
    //     std::cout
    //         << "WARNING: at least a facet in 'facets_on_input' is not coplanar with any facet in
    //         "
    //            "triangle_indexes with WMTK::RATIONAL"
    //         << std::endl;
    // }

    // v_rational.reserve(embedded_vertices.size()/3);
    // std::cout << "embed vertices size: " << embedded_vertices.size() << std::endl;
    // std::cout << "embed facets size: " << embedded_facets.size() << std::endl;
    // std::cout << "embed cells size: " << embedded_cells.size() << std::endl;
    // std::cout << "embed facet on input size: " << embedded_facets_on_input.size() << std::endl;

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

    // check polygon face validity
    // only for debug use
    // for (int i = 0; i < polygon_faces.size(); i++) {
    //     std::vector<tetwild::Vector3r> polyface_coordinates;
    //     for (int j = 0; j < polygon_faces[i].size(); j++) {
    //         polyface_coordinates.push_back(v_rational[polygon_faces[i][j]]);
    //     }
    //     if (!check_polygon_face_validity(polyface_coordinates)) {
    //         std::cout << "polygon face invalid!!: " << i << std::endl;
    //     }
    // }

    std::vector<bool> polygon_faces_on_input_surface(polygon_faces.size(), false);
    // for (int i = 0; i < polygon_faces.size(); i++) {
    //     polygon_faces_on_input_surface[i] = false;
    // }
    for (int i = 0; i < embedded_facets_on_input.size(); i++) {
        polygon_faces_on_input_surface[embedded_facets_on_input[i]] = true;
    }

    // output_embedded_polygon_mesh(
    //     "embedded_polygon_mesh.txt",
    //     v_rational,
    //     polygon_faces,
    //     polygon_cells,
    //     polygon_faces_on_input_surface);

    // output surface polygon
    output_embedded_polygon_surface_mesh(
        "surface_polygon_mesh_after_insertion.obj",
        v_rational,
        polygon_faces,
        polygon_faces_on_input_surface);

    // test mqz output

    // std::cout << "------mpz output------" << std::endl;
    // std::cout << "------1------" << std::endl;
    // std::cout << v_rational[15][0].to_double() << std::endl;
    // std::cout << "------2------" << std::endl;
    // std::cout << v_rational[15][0].get_str() << std::endl;
    // std::cout << "------3------" << std::endl;
    // std::cout << v_rational[15][0].get_num_str() << std::endl;
    // std::cout << v_rational[15][0].get_den_str() << std::endl;
    // // exit(0);
    // // std::cout << v_rational[15][0].get_num_str() << std::endl;
    // // std::cout << v_rational[15][0].get_den_str() << std::endl;
    // std::cout << "------mpz output end------" << std::endl;

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
            // std::cout<<std::endl<<"polyface: ";
            // for (int j=0; j<polygon_face.size(); j++){
            // std::cout<<polygon_face[j]<<" ";
            // }
            // std::cout<<std::endl<<"coords: "<<std::endl;
            for (int j = 0; j < polygon_faces[i].size(); j++) {
                poly_coordinates.push_back(v_rational[polygon_face[j]]);
                // std::cout<<v_rational[polygon_face[j]][0]<<"
                // "<<v_rational[polygon_face[j]][1]<<"
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

    std::cout << "poly_cnt:" << poly_cnt << std::endl;

    // std::cout << triangulated_faces.size() << std::endl;
    // int sum = 0;
    // for (int i = 0; i < map_poly_to_tri_face.size(); i++) {
    //     sum += map_poly_to_tri_face[i].size();
    // }
    // std::cout << sum << std::endl;

    // std::cout << "triangulated faces on input vector size: " <<
    // triangulated_faces_on_input.size()
    //           << std::endl;
    // int on_sur_cnt = 0;
    // for (int i = 0; i < triangulated_faces_on_input.size(); i++) {
    //     if (triangulated_faces_on_input[i]) on_sur_cnt++;
    // }
    // std::cout << "triangulated faces on input: " << on_sur_cnt << std::endl;

    std::cout << "finish triangulation" << std::endl;

    std::cout << "vertice before tetra num: " << v_rational.size() << std::endl;

    // invert the orientation of all the triangles
    // maybe commented
    // debug code?
    // int cnt_inverted_tri = 0;
    // for (size_t i = 0; i < triangulated_faces.size(); i++) {
    //     triangulated_faces[i] = {
    //         {triangulated_faces[i][0], triangulated_faces[i][2], triangulated_faces[i][1]}};
    // }

    // wmtk::logger().info(
    //     "inverted tri num: {}, total num: {}",
    //     cnt_inverted_tri,
    //     triangulated_faces.size());

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

        if (a.dot(b.cross(c)) > 0)
            continue;
        else
            return false;
    }
    return true;
}

void tetwild::TetWild::init_from_Volumeremesher(
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::array<size_t, 3>>& facets,
    const std::vector<bool>& is_v_on_input,
    const std::vector<std::array<size_t, 4>>& tets,
    const std::vector<bool>& tet_face_on_input_surface)
{
    init_with_isolated_vertices(v_rational.size(), tets);
    assert(check_mesh_connectivity_validity());

    m_vertex_attribute.m_attributes.resize(v_rational.size());
    m_tet_attribute.m_attributes.resize(tets.size());
    m_face_attribute.m_attributes.resize(tets.size() * 4);

    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_pos = v_rational[i];
        // wmtk::logger().info("rational: {}", m_vertex_attribute[i].m_pos);
        m_vertex_attribute[i].m_posf = to_double(v_rational[i]);
        // wmtk::logger().info("double: {}", m_vertex_attribute[i].m_posf);
    }

    // check here
    for (size_t i = 0; i < tet_face_on_input_surface.size(); i++) {
        if (tet_face_on_input_surface[i]) m_face_attribute[i].m_is_surface_fs = 1;
    }

    auto faces = get_faces();
    std::cout << "faces size: " << faces.size() << std::endl;
    for (auto f : faces) {
        auto fid = f.fid(*this);
        auto v1 = f.vid(*this);
        auto v2 = f.switch_vertex(*this).vid(*this);
        auto v3 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
        if (m_face_attribute[f.fid(*this)].m_is_surface_fs == 1) {
            m_vertex_attribute[v1].m_is_on_surface = true;
            m_vertex_attribute[v2].m_is_on_surface = true;
            m_vertex_attribute[v3].m_is_on_surface = true;
        }
    }

    // track bounding box

    for (size_t i = 0; i < faces.size(); i++) {
        auto fid_test = faces[i].fid(*this);
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
        // auto fid = faces[i].fid(*this);
        if (on_bbox < 0) continue;
        auto fid = faces[i].fid(*this);
        m_face_attribute[fid].m_is_bbox_fs = on_bbox;

        for (size_t vid : vids) {
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
        }
    }

    for_each_vertex(
        [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });


    // test code
    for (auto f : get_faces()) {
        auto fid = f.fid(*this);
    }

    std::cout << "pass test fid" << std::endl;

    // track open boundaries
    find_open_boundary();

    int open_boundary_cnt = 0;
    for (auto e : get_edges()) {
        if (is_open_boundary_edge(e)) open_boundary_cnt++;
    }
    wmtk::logger().info("#open boundary edges: {}", open_boundary_cnt);


    // enable on for tests
    // debug codes
    // if (!check_nondegenerate_tets()) {
    //     std::cout << "find tet with <=0 volume!" << std::endl;
    // }

    // if (!check_mesh_connectivity_validity()) {
    //     std::cout << "invalid mesh connectivity!" << std::endl;
    // }
    if (m_params.preserve_geometry) {
        for (auto f : get_faces()) {
            size_t f_id_for_geo = f.fid(*this);
            if (m_face_attribute[f_id_for_geo].m_is_surface_fs) {
                // std::cout << f_id_for_geo << std::endl;
                size_t in_collection = find_collection_for_tracked_surface(f);

                // debug code
                if (in_collection == -1) {
                    std::cout << "track surface cannot find matching input collection" << std::endl;
                }
                if (in_collection == -2) {
                    std::cout << "track surface cannot find coplanar matching input collection"
                              << std::endl;
                }
                if (in_collection == -3) {
                    std::cout << "track surface cannot find containment matching input collection"
                              << std::endl;
                }

                m_face_attribute[f_id_for_geo].from_input_collection_id = in_collection;
                m_face_attribute[f_id_for_geo].from_input_nearly_collection_id =
                    triangle_collections_from_input_surface.exact_to_nearly_map[in_collection];

                // for edge param
                // collection id to tracked faces id
                // may not need this
                // triangle_collections_from_input_surface.collections[in_collection]
                //     .tracked_face_ids.push_back(f_id_for_geo);
                // triangle_collections_from_input_surface
                //     .nearly_coplanar_collections[m_face_attribute[f_id_for_geo]
                //                                      .from_input_nearly_collection_id]
                //     .tracked_face_ids.push_back(f_id_for_geo);

                size_t f_v1 = f.vid(*this);
                size_t f_v2 = f.switch_vertex(*this).vid(*this);
                size_t f_v3 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
                // v1
                // exact


                if (triangle_collections_from_input_surface.collections[in_collection].effective) {
                    m_vertex_attribute[f_v1].face_param_type.push_back(in_collection);
                    m_vertex_attribute[f_v2].face_param_type.push_back(in_collection);
                    m_vertex_attribute[f_v3].face_param_type.push_back(in_collection);
                }

                m_vertex_attribute[f_v1].face_param_type_with_ineffective.push_back(in_collection);
                m_vertex_attribute[f_v2].face_param_type_with_ineffective.push_back(in_collection);
                m_vertex_attribute[f_v3].face_param_type_with_ineffective.push_back(in_collection);

                if (triangle_collections_from_input_surface
                        .nearly_coplanar_collections[triangle_collections_from_input_surface
                                                         .exact_to_nearly_map[in_collection]]
                        .effective) {
                    m_vertex_attribute[f_v1].face_nearly_param_type.push_back(
                        triangle_collections_from_input_surface.exact_to_nearly_map[in_collection]);
                    m_vertex_attribute[f_v2].face_nearly_param_type.push_back(
                        triangle_collections_from_input_surface.exact_to_nearly_map[in_collection]);
                    m_vertex_attribute[f_v3].face_nearly_param_type.push_back(
                        triangle_collections_from_input_surface.exact_to_nearly_map[in_collection]);
                }

                m_vertex_attribute[f_v1].face_nearly_param_type_with_ineffective.push_back(
                    triangle_collections_from_input_surface.exact_to_nearly_map[in_collection]);
                m_vertex_attribute[f_v2].face_nearly_param_type_with_ineffective.push_back(
                    triangle_collections_from_input_surface.exact_to_nearly_map[in_collection]);
                m_vertex_attribute[f_v3].face_nearly_param_type_with_ineffective.push_back(
                    triangle_collections_from_input_surface.exact_to_nearly_map[in_collection]);


                // wmtk::Rational u1, v1;
                // u1 = (m_vertex_attribute[f_v1].m_pos -
                //       triangle_collections_from_input_surface.collections[in_collection].a_pos)
                //          .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                   .param_u);
                // v1 = (m_vertex_attribute[f_v1].m_pos -
                //       triangle_collections_from_input_surface.collections[in_collection].a_pos)
                //          .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                   .param_v);
                // m_vertex_attribute[f_v1].uv_coords.push_back(std::make_pair(u1, v1));
                // m_vertex_attribute[f_v1].uv_coords_f.push_back(
                //     std::make_pair(u1.to_double(), v1.to_double()));

                // nearly

                // m_vertex_attribute[f_v1].face_nearly_param_type.push_back(
                //     triangle_collections_from_input_surface.exact_to_nearly_map[in_collection]);


                // double u1_f, v1_f;
                // u1_f = (m_vertex_attribute[f_v1].m_posf -
                //         triangle_collections_from_input_surface.collections[in_collection].a_pos_f)
                //            .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                     .param_u_f);
                // v1_f = (m_vertex_attribute[f_v1].m_posf -
                //         triangle_collections_from_input_surface.collections[in_collection].a_pos_f)
                //            .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                     .param_v_f);
                // m_vertex_attribute[f_v1].uv_nearly_f.push_back(std::make_pair(u1_f, v1_f));


                // v2
                // exact

                // m_vertex_attribute[f_v2].face_param_type.push_back(in_collection);

                // wmtk::Rational u2, v2;
                // u2 = (m_vertex_attribute[f_v2].m_pos -
                //       triangle_collections_from_input_surface.collections[in_collection].a_pos)
                //          .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                   .param_u);
                // v2 = (m_vertex_attribute[f_v2].m_pos -
                //       triangle_collections_from_input_surface.collections[in_collection].a_pos)
                //          .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                   .param_v);
                // m_vertex_attribute[f_v2].uv_coords.push_back(std::make_pair(u2, v2));
                // m_vertex_attribute[f_v2].uv_coords_f.push_back(
                //     std::make_pair(u2.to_double(), v2.to_double()));

                // nearly
                // m_vertex_attribute[f_v2].face_nearly_param_type.push_back(
                //     triangle_collections_from_input_surface.exact_to_nearly_map[in_collection]);

                // double u2_f, v2_f;
                // u2_f = (m_vertex_attribute[f_v2].m_posf -
                //         triangle_collections_from_input_surface.collections[in_collection].a_pos_f)
                //            .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                     .param_u_f);
                // v2_f = (m_vertex_attribute[f_v2].m_posf -
                //         triangle_collections_from_input_surface.collections[in_collection].a_pos_f)
                //            .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                     .param_v_f);
                // m_vertex_attribute[f_v2].uv_nearly_f.push_back(std::make_pair(u2_f, v2_f));

                // v3
                // exact

                // m_vertex_attribute[f_v3].face_param_type.push_back(in_collection);

                // wmtk::Rational u3, v3;
                // u3 = (m_vertex_attribute[f_v3].m_pos -
                //       triangle_collections_from_input_surface.collections[in_collection].a_pos)
                //          .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                   .param_u);
                // v3 = (m_vertex_attribute[f_v3].m_pos -
                //       triangle_collections_from_input_surface.collections[in_collection].a_pos)
                //          .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                   .param_v);
                // m_vertex_attribute[f_v3].uv_coords.push_back(std::make_pair(u3, v3));
                // m_vertex_attribute[f_v3].uv_coords_f.push_back(
                //     std::make_pair(u3.to_double(), v3.to_double()));

                // nearly
                // m_vertex_attribute[f_v3].face_nearly_param_type.push_back(
                //     triangle_collections_from_input_surface.exact_to_nearly_map[in_collection]);

                // double u3_f, v3_f;
                // u3_f = (m_vertex_attribute[f_v3].m_posf -
                //         triangle_collections_from_input_surface.collections[in_collection].a_pos_f)
                //            .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                     .param_u_f);
                // v3_f = (m_vertex_attribute[f_v3].m_posf -
                //         triangle_collections_from_input_surface.collections[in_collection].a_pos_f)
                //            .dot(triangle_collections_from_input_surface.collections[in_collection]
                //                     .param_v_f);
                // m_vertex_attribute[f_v3].uv_nearly_f.push_back(std::make_pair(u3_f, v3_f));
            }
        }

        auto vs = get_vertices();

        for (auto v : vs) {
            if (m_vertex_attribute[v.vid(*this)].m_is_on_surface) {
                wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].face_param_type);
                wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].face_nearly_param_type);
                wmtk::vector_unique(
                    m_vertex_attribute[v.vid(*this)].face_param_type_with_ineffective);
                wmtk::vector_unique(
                    m_vertex_attribute[v.vid(*this)].face_nearly_param_type_with_ineffective);
                // if (m_vertex_attribute[v.vid(*this)].face_param_type.size() == 0) {
                //     std::cout << "0 size face param!" << std::endl;
                // }
                // if (m_vertex_attribute[v.vid(*this)].face_nearly_param_type.size() == 0) {
                //     std::cout << "0 size nearly face param!" << std::endl;
                // }
            }
        }

        if (m_params.preserve_geometry && !check_vertex_param_type()) {
            std::cout << "missing param at here!!!!!!!!" << std::endl;
        }

        // get vertices on nearly coplanar collection boundaries or open boundaries
        int cnt_on_collection_boundary = 0;
        std::ofstream file("debug1.obj");
        int xxx = 1;
        for (auto v : vs) {
            if (!m_vertex_attribute[v.vid(*this)].m_is_on_surface) continue;
            if (m_vertex_attribute[v.vid(*this)].face_nearly_param_type.size() > 1 ||
                (m_vertex_attribute[v.vid(*this)].face_nearly_param_type.size() == 1 &&
                 m_vertex_attribute[v.vid(*this)].face_nearly_param_type_with_ineffective.size() >
                     1) ||
                (m_vertex_attribute[v.vid(*this)].m_is_on_open_boundary &&
                 m_vertex_attribute[v.vid(*this)].face_nearly_param_type.size() > 0)) {
                m_vertex_attribute[v.vid(*this)].is_on_collection_boundary = true;
                std::cout << v.vid(*this) << " ";
                file << "v " << m_vertex_attribute[v.vid(*this)].m_posf[0] << " "
                     << m_vertex_attribute[v.vid(*this)].m_posf[1] << " "
                     << m_vertex_attribute[v.vid(*this)].m_posf[2] << std::endl;
                xxx++;
                cnt_on_collection_boundary++;
            }
        }

        std::cout << std::endl;

        std::cout << "cnt v on colleciton boundary: " << cnt_on_collection_boundary << std::endl;

        // get connectivity of vertices on collection boundaries
        auto edges = get_edges();
        // std::vector<bool> is_collection_boundary_edge(edges.size(), false);
        int cnt_collection_be = 0;

        for (auto v : vs) {
            if (!m_vertex_attribute[v.vid(*this)].is_on_collection_boundary) continue;
            auto one_ring_v = get_one_ring_vertices_for_vertex(v);
            for (auto v_or : one_ring_v) {
                if (m_vertex_attribute[v_or.vid(*this)].is_on_collection_boundary) {
                    auto e = tuple_from_edge({{v.vid(*this), v_or.vid(*this)}});
                    if (is_edge_on_surface(e)) {
                        auto incident_tets = get_incident_tets_for_edge(e);
                        std::vector<size_t> incident_surface_faces;
                        for (auto t : incident_tets) {
                            std::vector<Tuple> f(4);
                            f[0] = t;
                            f[1] = t.switch_face(*this);
                            f[2] = t.switch_edge(*this).switch_face(*this);
                            f[3] = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);

                            for (int h = 0; h < 4; h++) {
                                if (!m_face_attribute[f[h].fid(*this)].m_is_surface_fs) continue;
                                auto vs = get_face_vertices(f[h]);
                                std::array<size_t, 3> vids = {
                                    {vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
                                if (std::find(vids.begin(), vids.end(), v.vid(*this)) !=
                                        vids.end() &&
                                    std::find(vids.begin(), vids.end(), v_or.vid(*this)) !=
                                        vids.end()) {
                                    incident_surface_faces.push_back(f[h].fid(*this));
                                }
                            }
                        }
                        wmtk::vector_unique(incident_surface_faces);

                        bool boundary_flag = false;
                        for (int k = 0; k < int(incident_surface_faces.size()) - 1; k++) {
                            if (m_face_attribute[incident_surface_faces[k]]
                                    .from_input_nearly_collection_id !=
                                m_face_attribute[incident_surface_faces[k + 1]]
                                    .from_input_nearly_collection_id) {
                                boundary_flag = true;
                                break;
                            }
                        }

                        if (!boundary_flag) continue;
                        // std::cout << v.vid(*this) << " " << v_or.vid(*this) << std::endl;
                        m_vertex_attribute[v.vid(*this)]
                            .connected_collection_boundary_vertices.push_back(v_or.vid(*this));
                        // is_collection_boundary_edge[e.eid(*this)] = true;
                        // file << "v " << m_vertex_attribute[v.vid(*this)].m_posf[0] << " "
                        //      << m_vertex_attribute[v.vid(*this)].m_posf[1] << " "
                        //      << m_vertex_attribute[v.vid(*this)].m_posf[2] << std::endl;
                        // file << "v " << m_vertex_attribute[v_or.vid(*this)].m_posf[0] << " "
                        //      << m_vertex_attribute[v_or.vid(*this)].m_posf[1] << " "
                        //      << m_vertex_attribute[v_or.vid(*this)].m_posf[2] << std::endl;
                        // file << "l " << xxx << " " << xxx + 1 << std::endl;
                        // xxx += 2;
                        cnt_collection_be++;
                    }
                }
            }
        }
        // std::cout << "#collection be: " << cnt_collection_be << std::endl;

        for (auto v : get_vertices()) {
            if (m_vertex_attribute[v.vid(*this)].is_on_collection_boundary) {
                std::cout << m_vertex_attribute[v.vid(*this)]
                                 .connected_collection_boundary_vertices.size()
                          << ": ";

                for (int j = 0;
                     j <
                     m_vertex_attribute[v.vid(*this)].connected_collection_boundary_vertices.size();
                     j++) {
                    std::cout << m_vertex_attribute[v.vid(*this)]
                                     .connected_collection_boundary_vertices[j]
                              << " ";
                }
                std::cout << std::endl;
            }
        }


        // debug code
        // int cnt_collection_boundary_edge = 0;
        // for (bool flag : is_collection_boundary_edge) {
        //     if (flag) cnt_collection_boundary_edge++;
        // }
        // std::cout << "#collection boundary edges: " << cnt_collection_boundary_edge << std::endl;

        // std::vector<bool> visited(edges.size(), false);
        std::map<std::pair<size_t, size_t>, bool> visited;
        double theta = 5;
        double tol = std::cos(theta / 180 * M_PI);
        for (size_t i = 0; i < edges.size(); i++) {
            // check is collection boundary edge
            size_t v1 = edges[i].vid(*this);
            size_t v2 = edges[i].switch_vertex(*this).vid(*this);
            if (visited.find(std::make_pair(v1, v2)) != visited.end()) continue;
            if (std::find(
                    m_vertex_attribute[v1].connected_collection_boundary_vertices.begin(),
                    m_vertex_attribute[v1].connected_collection_boundary_vertices.end(),
                    v2) == m_vertex_attribute[v1].connected_collection_boundary_vertices.end())
                continue;

            // auto e = edges[i];
            // auto incident_tets = get_incident_tets_for_edge(e);
            // std::vector<size_t> incident_surface_faces;
            // for (auto t : incident_tets) {
            //     std::vector<Tuple> f(4);
            //     f[0] = t;
            //     f[1] = t.switch_face(*this);
            //     f[2] = t.switch_edge(*this).switch_face(*this);
            //     f[3] = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);

            //     for (int h = 0; h < 4; h++) {
            //         if (!m_face_attribute[f[h].fid(*this)].m_is_surface_fs) continue;
            //         auto vs = get_face_vertices(f[h]);
            //         std::array<size_t, 3> vids = {
            //             {vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
            //         if (std::find(vids.begin(), vids.end(), v1) != vids.end() &&
            //             std::find(vids.begin(), vids.end(), v2) != vids.end()) {
            //             incident_surface_faces.push_back(f[h].fid(*this));
            //         }
            //     }
            // }
            // wmtk::vector_unique(incident_surface_faces);

            // bool boundary_flag = false;
            // for (int k = 0; k < int(incident_surface_faces.size()) - 1; k++) {
            //     if (m_face_attribute[incident_surface_faces[k]].from_input_nearly_collection_id
            //     !=
            //         m_face_attribute[incident_surface_faces[k +
            //         1]].from_input_nearly_collection_id) boundary_flag = true;
            // }

            // if (!boundary_flag) continue;

            file << "v " << m_vertex_attribute[v1].m_posf[0] << " "
                 << m_vertex_attribute[v1].m_posf[1] << " " << m_vertex_attribute[v1].m_posf[2]
                 << std::endl;
            file << "v " << m_vertex_attribute[v2].m_posf[0] << " "
                 << m_vertex_attribute[v2].m_posf[1] << " " << m_vertex_attribute[v2].m_posf[2]
                 << std::endl;
            file << "l " << xxx << " " << xxx + 1 << std::endl;
            xxx += 2;


            visited[std::make_pair(v1, v2)] = true;
            visited[std::make_pair(v2, v1)] = true;

            // compute a new edge param


            Vector3d direction =
                (m_vertex_attribute[v1].m_posf - m_vertex_attribute[v2].m_posf).normalized();

            edge_parametrization ep;
            ep.direction = direction;
            ep.origin = m_vertex_attribute[v1].m_posf;
            size_t ep_id = edge_params.size();

            std::vector<size_t> edge_vertices;
            edge_vertices.push_back(v1);
            edge_vertices.push_back(v2);

            // expand through v1;
            size_t endpoint1 = v1;
            size_t previous = v2;
            while (m_vertex_attribute[endpoint1].connected_collection_boundary_vertices.size() ==
                   2) {
                // find next point
                size_t next =
                    m_vertex_attribute[endpoint1].connected_collection_boundary_vertices[0];
                if (next == previous)
                    next = m_vertex_attribute[endpoint1].connected_collection_boundary_vertices[1];

                // auto candidate_e1 = tuple_from_edge({{next, endpoint1}});

                // auto incident_tets_d1 = get_incident_tets_for_edge(candidate_e1);
                // std::vector<size_t> incident_surface_faces_d1;
                // for (auto t : incident_tets_d1) {
                //     std::vector<Tuple> f(4);
                //     f[0] = t;
                //     f[1] = t.switch_face(*this);
                //     f[2] = t.switch_edge(*this).switch_face(*this);
                //     f[3] = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);

                //     for (int h = 0; h < 4; h++) {
                //         if (!m_face_attribute[f[h].fid(*this)].m_is_surface_fs) continue;
                //         auto vs = get_face_vertices(f[h]);
                //         std::array<size_t, 3> vids = {
                //             {vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
                //         if (std::find(vids.begin(), vids.end(), endpoint1) != vids.end() &&
                //             std::find(vids.begin(), vids.end(), next) != vids.end()) {
                //             incident_surface_faces_d1.push_back(f[h].fid(*this));
                //         }
                //     }
                // }
                // wmtk::vector_unique(incident_surface_faces_d1);

                // bool boundary_flag_d1 = false;
                // for (int k = 0; k < int(incident_surface_faces_d1.size()) - 1; k++) {
                //     if (m_face_attribute[incident_surface_faces_d1[k]]
                //             .from_input_nearly_collection_id !=
                //         m_face_attribute[incident_surface_faces_d1[k + 1]]
                //             .from_input_nearly_collection_id) {
                //         boundary_flag_d1 = true;
                //         break;
                //     }
                // }

                // if (!boundary_flag_d1) {
                //     std::cout << endpoint1 << std::endl;
                //     break;
                // }

                // check if nearly colinear
                Vector3d next_direction =
                    m_vertex_attribute[next].m_posf - m_vertex_attribute[endpoint1].m_posf;
                if (next_direction.dot(direction) >
                    tol * (next_direction.norm() * direction.norm())) {
                    visited[std::make_pair(next, endpoint1)] = true;
                    visited[std::make_pair(endpoint1, next)] = true;
                    edge_vertices.push_back(next);

                    file << "v " << m_vertex_attribute[next].m_posf[0] << " "
                         << m_vertex_attribute[next].m_posf[1] << " "
                         << m_vertex_attribute[next].m_posf[2] << std::endl;
                    file << "v " << m_vertex_attribute[endpoint1].m_posf[0] << " "
                         << m_vertex_attribute[endpoint1].m_posf[1] << " "
                         << m_vertex_attribute[endpoint1].m_posf[2] << std::endl;
                    file << "l " << xxx << " " << xxx + 1 << std::endl;
                    xxx += 2;

                    // go to next
                    previous = endpoint1;
                    endpoint1 = next;


                    // debug code

                } else {
                    std::cout << endpoint1 << std::endl;

                    break;
                }
            }

            size_t endpoint2 = v2;
            previous = v1;

            while (m_vertex_attribute[endpoint2].connected_collection_boundary_vertices.size() ==
                   2) {
                // find next point
                size_t next =
                    m_vertex_attribute[endpoint2].connected_collection_boundary_vertices[0];
                if (next == previous)
                    next = m_vertex_attribute[endpoint2].connected_collection_boundary_vertices[1];

                // auto candidate_e2 = tuple_from_edge({{next, endpoint2}});

                // auto incident_tets_d2 = get_incident_tets_for_edge(candidate_e2);
                // std::vector<size_t> incident_surface_faces_d2;
                // for (auto t : incident_tets_d2) {
                //     std::vector<Tuple> f(4);
                //     f[0] = t;
                //     f[1] = t.switch_face(*this);
                //     f[2] = t.switch_edge(*this).switch_face(*this);
                //     f[3] = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);

                //     for (int h = 0; h < 4; h++) {
                //         if (!m_face_attribute[f[h].fid(*this)].m_is_surface_fs) continue;
                //         auto vs = get_face_vertices(f[h]);
                //         std::array<size_t, 3> vids = {
                //             {vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
                //         if (std::find(vids.begin(), vids.end(), next) != vids.end() &&
                //             std::find(vids.begin(), vids.end(), endpoint2) != vids.end()) {
                //             incident_surface_faces_d2.push_back(f[h].fid(*this));
                //         }
                //     }
                // }
                // wmtk::vector_unique(incident_surface_faces_d2);

                // bool boundary_flag_d2 = false;
                // for (int k = 0; k < int(incident_surface_faces_d2.size()) - 1; k++) {
                //     if (m_face_attribute[incident_surface_faces_d2[k]]
                //             .from_input_nearly_collection_id !=
                //         m_face_attribute[incident_surface_faces_d2[k + 1]]
                //             .from_input_nearly_collection_id) {
                //         boundary_flag_d2 = true;

                //         break;
                //     }
                // }

                // if (!boundary_flag_d2) {
                //     std::cout << "endpoint2:" << endpoint2 << std::endl;
                //     break;
                // }

                // check if nearly colinear
                Vector3d next_direction =
                    m_vertex_attribute[next].m_posf - m_vertex_attribute[endpoint2].m_posf;
                if (next_direction.dot(-direction) >
                    tol * (next_direction.norm() * direction.norm())) {
                    visited[std::make_pair(next, endpoint2)] = true;
                    visited[std::make_pair(endpoint2, next)] = true;
                    edge_vertices.push_back(next);

                    // debug code
                    file << "v " << m_vertex_attribute[next].m_posf[0] << " "
                         << m_vertex_attribute[next].m_posf[1] << " "
                         << m_vertex_attribute[next].m_posf[2] << std::endl;
                    file << "v " << m_vertex_attribute[endpoint2].m_posf[0] << " "
                         << m_vertex_attribute[endpoint2].m_posf[1] << " "
                         << m_vertex_attribute[endpoint2].m_posf[2] << std::endl;
                    file << "l " << xxx << " " << xxx + 1 << std::endl;
                    xxx += 2;
                    //
                    // go to next
                    previous = endpoint2;
                    endpoint2 = next;
                } else {
                    std::cout << "noncolinear stop at endpoint2:" << endpoint2 << std::endl;
                    break;
                }
            }
            //
            // set edge params
            for (auto v : edge_vertices) {
                m_vertex_attribute[v].in_edge_param.push_back(ep_id);
            }
            m_vertex_attribute[endpoint1].is_freezed = true;
            m_vertex_attribute[endpoint2].is_freezed = true;

            edge_params.push_back(ep);
        }
    }

    // test code
    for (auto v : get_vertices()) {
        if (m_vertex_attribute[v.vid(*this)].is_on_collection_boundary) {
            if (m_vertex_attribute[v.vid(*this)].in_edge_param.size() < 1)
                std::cout << "missing edge param for collection boundary vertices!" << std::endl;
        }
    }

    int cnt_freeze = 0;
    for (auto v : get_vertices()) {
        if (m_vertex_attribute[v.vid(*this)].is_on_collection_boundary) {
            if (m_vertex_attribute[v.vid(*this)].is_freezed) cnt_freeze++;
        }
    }
    std::cout << "#freezed vertices: " << cnt_freeze << std::endl;

    std::ofstream freezed_v("freezed_vertices.obj");
    for (auto v : get_vertices()) {
        if (m_vertex_attribute[v.vid(*this)].is_freezed) {
            freezed_v << "v " << m_vertex_attribute[v.vid(*this)].m_posf[0] << " "
                      << m_vertex_attribute[v.vid(*this)].m_posf[1] << " "
                      << m_vertex_attribute[v.vid(*this)].m_posf[2] << std::endl;
        }
    }


    std::cout << "#edge_params: " << edge_params.size() << std::endl;


    if (m_params.preserve_geometry && !check_vertex_param_type()) {
        std::cout << "missing param!!!!!!!!" << std::endl;
    }

    // check 174 1444
    // std::cout << "v 174:" << std::endl;
    // std::cout << "coord: " << m_vertex_attribute[174].m_posf[0] << " "
    //           << m_vertex_attribute[174].m_posf[1] << " " << m_vertex_attribute[174].m_posf[2]
    //           << std::endl;
    // std::cout << "coord rational: " << m_vertex_attribute[174].m_pos[0].to_double() << " "
    //           << m_vertex_attribute[174].m_pos[1].to_double() << " "
    //           << m_vertex_attribute[174].m_pos[2].to_double() << std::endl;
    // std::cout << "is freezed: " << m_vertex_attribute[174].is_freezed << std::endl;
    // std::cout << "effective face params: ";
    // for (int i = 0; i < m_vertex_attribute[174].face_nearly_param_type.size(); i++) {
    //     std::cout << m_vertex_attribute[174].face_nearly_param_type[i] << " ";
    // }
    // std::cout << std::endl;
    // std::cout << "all face params: ";
    // for (int i = 0; i < m_vertex_attribute[174].face_nearly_param_type_with_ineffective.size();
    //      i++) {
    //     std::cout << m_vertex_attribute[174].face_nearly_param_type_with_ineffective[i] << " ";
    // }
    // std::cout << std::endl;
    // std::cout << "edge params: ";
    // for (int i = 0; i < m_vertex_attribute[174].in_edge_param.size(); i++) {
    //     std::cout << m_vertex_attribute[174].in_edge_param[i] << " ";
    // }
    // std::cout << std::endl;


    // std::cout << "v 1444:" << std::endl;
    // std::cout << "coord: " << m_vertex_attribute[1444].m_posf[0] << " "
    //           << m_vertex_attribute[1444].m_posf[1] << " " << m_vertex_attribute[1444].m_posf[2]
    //           << std::endl;
    // std::cout << "coord rational: " << m_vertex_attribute[1444].m_pos[0].to_double() << " "
    //           << m_vertex_attribute[1444].m_pos[1].to_double() << " "
    //           << m_vertex_attribute[1444].m_pos[2].to_double() << std::endl;

    // std::cout << "is freezed: " << m_vertex_attribute[1444].is_freezed << std::endl;
    // std::cout << "effective face params: ";
    // for (int i = 0; i < m_vertex_attribute[1444].face_nearly_param_type.size(); i++) {
    //     std::cout << m_vertex_attribute[1444].face_nearly_param_type[i] << " ";
    // }
    // std::cout << std::endl;
    // std::cout << "all face params: ";
    // for (int i = 0; i < m_vertex_attribute[1444].face_nearly_param_type_with_ineffective.size();
    //      i++) {
    //     std::cout << m_vertex_attribute[1444].face_nearly_param_type_with_ineffective[i] << " ";
    // }
    // std::cout << std::endl;
    // std::cout << "edge params: ";
    // for (int i = 0; i < m_vertex_attribute[1444].in_edge_param.size(); i++) {
    //     std::cout << m_vertex_attribute[1444].in_edge_param[i] << " ";
    // }
    // std::cout << std::endl;


    // // rounding
    // std::atomic_int cnt_round(0);
    // std::atomic_int cnt_valid(0);

    // auto vertices = get_vertices();
    // for (auto v : vertices) {
    //     // debug code
    //     if (v.is_valid(*this)) cnt_valid++;

    //     if (round(v)) cnt_round++;
    // }

    // wmtk::logger().info("cnt_round {}/{}", cnt_round, cnt_valid);
    // // rounding
    std::atomic_int cnt_round(0);
    std::atomic_int cnt_valid(0);

    auto vertices = get_vertices();
    for (auto v : vertices) {
        // debug code
        if (v.is_valid(*this)) cnt_valid++;

        if (round(v)) cnt_round++;
    }

    wmtk::logger().info("cnt_round {}/{}", cnt_round, cnt_valid);

    // init qualities
    auto& m = *this;
    for_each_tetra([&m](auto& t) { m.m_tet_attribute[t.tid(m)].m_quality = m.get_quality(t); });
}

// void tetwild::TetWild::init_from_file(std::string input_dir)
// {
//     std::ifstream input(input_dir);
//     size_t v_num, f_num, t_num;
//     input >> v_num >> f_num >> t_num;

//     std::vector<Vector3r> v_rational;
//     std::vector<std::array<size_t, 4>> tets;
//     std::vector<bool> is_v_on_surface;
//     std::vector<bool> is_f_on_surface;
//     std::vector<int> is_f_on_bbox;

//     v_rational.reserve(v_num);
//     tets.reserve(t_num);
//     is_v_on_surface.reserve(v_num);
//     is_f_on_surface.reserve(f_num);
//     is_f_on_bbox.reserve(f_num);

//     for (size_t i = 0; i < v_num; i++) {
//         char type;
//         double x, y, z;
//         bool on_surface;
//         input >> type >> x >> y >> z >> on_surface;
//         Vector3d p(x, y, z);
//         v_rational.push_back(to_rational(p));
//         is_v_on_surface.push_back(on_surface);
//     }

//     for (size_t i = 0; i < f_num; i++) {
//         char type;
//         size_t v1, v2, v3;
//         bool on_surface;
//         int on_bbox;
//         input >> type >> v1 >> v2 >> v3 >> on_surface >> on_bbox;
//         is_f_on_surface.push_back(on_surface);
//         is_f_on_bbox.push_back(on_bbox);
//     }

//     for (size_t i = 0; i < t_num; i++) {
//         char type;
//         size_t v1, v2, v3, v4;
//         input >> type >> v1 >> v2 >> v3 >> v4;
//         if (v1 >= v_num || v2 >= v_num || v3 >= v_num || v4 >= v_num) {
//             std::cout << "wrong vertex id!!!" << std::endl;
//             // exit(0);
//         }
//         std::array<size_t, 4> tet = {{v1, v2, v3, v4}};
//         tets.push_back(tet);
//     }

//     init(v_num, tets);
//     m_vertex_attribute.m_attributes.resize(v_num);
//     m_tet_attribute.m_attributes.resize(tets.size());
//     m_face_attribute.m_attributes.resize(tets.size() * 4);

//     for (int i = 0; i < vert_capacity(); i++) {
//         m_vertex_attribute[i].m_pos = v_rational[i];
//         m_vertex_attribute[i].m_posf = to_double(v_rational[i]);
//     }

//     auto faces = get_faces();
//     if (faces.size() != f_num) {
//         std::cout << "wrong face size!!" << std::endl;
//         // exit(0);
//     }

//     for (size_t i = 0; i < faces.size(); i++) {
//         size_t f_id = faces[i].fid(*this);
//         m_face_attribute[f_id].m_is_surface_fs = is_f_on_surface[i];
//         // m_face_attribute[f_id].m_is_on_bbox = is_f_on_bbox[f_id];
//     }

//     // track bbox
//     for (size_t i = 0; i < faces.size(); i++) {
//         auto vs = get_face_vertices(faces[i]);
//         std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
//         int on_bbox = -1;
//         for (int k = 0; k < 3; k++) {
//             if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_min[k] &&
//                 m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_min[k] &&
//                 m_vertex_attribute[vids[2]].m_pos[k] == m_params.box_min[k]) {
//                 on_bbox = k * 2;
//                 break;
//             }
//             if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_max[k] &&
//                 m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_max[k] &&
//                 m_vertex_attribute[vids[2]].m_pos[k] == m_params.box_max[k]) {
//                 on_bbox = k * 2 + 1;
//                 break;
//             }
//         }
//         if (on_bbox < 0) continue;
//         auto fid = faces[i].fid(*this);
//         m_face_attribute[fid].m_is_bbox_fs = on_bbox;

//         for (size_t vid : vids) {
//             m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
//         }
//     }

//     for_each_vertex(
//         [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });


//     // set v surface and rounding
//     auto vertices = get_vertices();

//     std::atomic_int cnt_round(0);
//     for (auto v : vertices) {
//         size_t v_id = v.vid(*this);
//         m_vertex_attribute[v_id].m_is_on_surface = is_v_on_surface[v_id];
//         if (round(v)) cnt_round++;
//     }

//     wmtk::logger().info("cnt_round {}/{}", cnt_round, vert_capacity());

//     // init tet quality
//     auto& m = *this;
//     for_each_tetra([&m](auto& t) { m.m_tet_attribute[t.tid(m)].m_quality = m.get_quality(t); });


//     input.close();
// }

void tetwild::TetWild::find_open_boundary()
{
    auto fs = get_faces();
    std::cout << "fs size: " << fs.size() << std::endl;
    auto es = get_edges();
    std::vector<int> edge_on_open_boundary(6 * tet_capacity(), 0);

    // for open boundary envelope
    std::vector<Eigen::Vector3d> v_posf(vert_capacity());
    std::vector<Eigen::Vector3i> open_boundaries;

    for (size_t i = 0; i < vert_capacity(); i++) {
        v_posf[i] = m_vertex_attribute[i].m_posf;
    }

    for (auto f : fs) {
        auto fid = f.fid(*this);
        // std::cout << fid << ": ";
        if (!m_face_attribute[fid].m_is_surface_fs) continue;
        size_t eid1 = f.eid(*this);
        size_t eid2 = f.switch_edge(*this).eid(*this);
        size_t eid3 = f.switch_vertex(*this).switch_edge(*this).eid(*this);

        // std::cout << eid1 << " " << eid2 << " " << eid3 << std::endl;

        edge_on_open_boundary[eid1]++;
        edge_on_open_boundary[eid2]++;
        edge_on_open_boundary[eid3]++;
    }

    for (auto e : es) {
        // std::cout << edge_on_open_boundary[e.eid(*this)] << " ";
        if (edge_on_open_boundary[e.eid(*this)] != 1) continue;
        size_t v1 = e.vid(*this);
        size_t v2 = e.switch_vertex(*this).vid(*this);
        m_vertex_attribute[v1].m_is_on_open_boundary = true;
        m_vertex_attribute[v2].m_is_on_open_boundary = true;
        open_boundaries.push_back(Eigen::Vector3i(v1, v2, v1));
    }

    wmtk::logger().info("open boundary num: {}", open_boundaries.size());

    if (open_boundaries.size() == 0) return;

    // init open boundary envelope
    m_open_boundary_envelope.init(v_posf, open_boundaries, m_params.epsr * m_params.diag_l / 2.0);
    boundaries_tree.init(v_posf, open_boundaries, m_params.epsr * m_params.diag_l / 2.0);
}

bool tetwild::TetWild::is_open_boundary_edge(const Tuple& e)
{
    size_t v1 = e.vid(*this);
    size_t v2 = e.switch_vertex(*this).vid(*this);
    if (!m_vertex_attribute[v1].m_is_on_open_boundary ||
        !m_vertex_attribute[v2].m_is_on_open_boundary)
        return false;

    return !m_open_boundary_envelope.is_outside(
        {{m_vertex_attribute[v1].m_posf,
          m_vertex_attribute[v2].m_posf,
          m_vertex_attribute[v1].m_posf}});
}

int tetwild::TetWild::orient3D(
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
bool tetwild::TetWild::checkTrackedFaces(
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

int tetwild::TetWild::orient3D_wmtk_rational(
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
bool tetwild::TetWild::checkTrackedFaces_wmtk_rational(
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
