
#include "VolumeRemesherTriangleInsertion.hpp"
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/utils/orient.hpp>


// clang-format off
#include <VolumeRemesher/embed.h>
// clang-format on

#include <numeric>

namespace wmtk::utils {

template <class T>
void vector_unique(std::vector<T>& v)
{
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
}

int64_t v_index(const std::array<int64_t, 3>& resolution, const std::array<int64_t, 3>& index)
{
    return index[2] * (resolution[0] + 1) * (resolution[1] + 1) + index[1] * (resolution[0] + 1) +
           index[0];
}

void generate_background_mesh(
    const Vector3d& bbox_min,
    const Vector3d& bbox_max,
    const std::array<int64_t, 3>& resolution, // number of cells in each dimension
    RowVectors4l& background_TV,
    RowVectors3d& background_V)
{
    // resize matrices
    background_TV.resize(6 * resolution[0] * resolution[1] * resolution[2], 4);
    background_V.resize((resolution[0] + 1) * (resolution[1] + 1) * (resolution[2] + 1), 3);

    // fill in tv
    int64_t tet_index = 0;

    for (int64_t k = 0; k < resolution[2]; ++k) {
        for (int64_t j = 0; j < resolution[1]; ++j) {
            for (int64_t i = 0; i < resolution[0]; ++i) {
                int64_t cell_v[8] = {
                    v_index(resolution, {{i, j, k}}),
                    v_index(resolution, {{i + 1, j, k}}),
                    v_index(resolution, {{i + 1, j + 1, k}}),
                    v_index(resolution, {{i, j + 1, k}}),
                    v_index(resolution, {{i, j, k + 1}}),
                    v_index(resolution, {{i + 1, j, k + 1}}),
                    v_index(resolution, {{i + 1, j + 1, k + 1}}),
                    v_index(resolution, {{i, j + 1, k + 1}})};
                background_TV.row(tet_index++) << cell_v[0], cell_v[1], cell_v[3], cell_v[4];
                background_TV.row(tet_index++) << cell_v[5], cell_v[2], cell_v[6], cell_v[7];
                background_TV.row(tet_index++) << cell_v[4], cell_v[1], cell_v[5], cell_v[3];
                background_TV.row(tet_index++) << cell_v[4], cell_v[3], cell_v[7], cell_v[5];
                background_TV.row(tet_index++) << cell_v[3], cell_v[1], cell_v[5], cell_v[2];
                background_TV.row(tet_index++) << cell_v[2], cell_v[3], cell_v[7], cell_v[5];
            }
        }
    }

    assert(tet_index == 6 * resolution[0] * resolution[1] * resolution[2]);

    //  fill in v
    for (int64_t k = 0; k < resolution[2] + 1; ++k) {
        for (int64_t j = 0; j < resolution[1] + 1; ++j) {
            for (int64_t i = 0; i < resolution[0] + 1; ++i) {
                background_V.row(v_index(resolution, {{i, j, k}}))
                    << bbox_min[0] + (bbox_max[0] - bbox_min[0]) * i / resolution[0],
                    bbox_min[1] + (bbox_max[1] - bbox_min[1]) * j / resolution[1],
                    bbox_min[2] + (bbox_max[2] - bbox_min[2]) * k / resolution[2];
            }
        }
    }
}

std::vector<std::array<int64_t, 3>> triangulate_polygon_face(const std::vector<Vector3r>& points)
{
    // triangulate weak convex polygons
    std::vector<std::array<int64_t, 3>> triangulated_faces;

    std::vector<int64_t> points_vector(points.size());
    std::iota(std::begin(points_vector), std::end(points_vector), 0);

    Vector3r out = points[0];
    for (int64_t i = 1; i < points.size(); ++i) {
        for (int64_t d = 0; d < 3; ++d) {
            if (out[d] < points[i][d]) out[d] = points[i][d];
        }
    }

    for (int64_t d = 0; d < 3; ++d) {
        out[d].round();
        out[d] = out[d] - 1;
    }

    // find the first colinear ABC with nonlinear BCD and delete C from vector
    while (points_vector.size() > 3) {
        bool no_colinear = true;
        for (int64_t i = 0; i < points_vector.size(); ++i) {
            const int64_t next_i = (i + 1) % points_vector.size();
            const int64_t prev_i = (i + points_vector.size() - 1) % points_vector.size();
            const int64_t nextnext_i = (i + 2) % points_vector.size();

            const int64_t cur = points_vector[i];
            const int64_t next = points_vector[next_i];
            const int64_t prev = points_vector[prev_i];
            const int64_t nextnext = points_vector[nextnext_i];


            // TODO: check if orient2d == 0 works for check colinearity
            // if (orient2d(prev.first.data(), cur.first.data(), next.first.data()) == 0 &&
            //     orient2d(cur.first.data(), next.first.data(), nextnext.first.data()) != 0) {

            const Vector3r a = points[cur] - points[prev];
            const Vector3r b = points[next] - points[cur];
            const Vector3r c = points[nextnext] - points[next];

            // if (((a[0] * b[1] - a[1] * b[0]).get_sign() == 0 &&
            //      (a[1] * b[2] - a[2] * b[1]).get_sign() == 0 &&
            //      (a[0] * b[2] - a[2] * b[0]).get_sign() == 0) &&
            //     (((b[0] * c[1] - b[1] * c[0]).get_sign() != 0 ||
            //       (b[1] * c[2] - b[2] * c[1]).get_sign() != 0 ||
            //       (b[0] * c[2] - b[2] * c[0]).get_sign() != 0))) {
            if (wmtk_orient3d(points[prev], points[cur], points[next], out) == 0 &&
                wmtk_orient3d(points[cur], points[next], points[nextnext], out) == 0) {
                no_colinear = false;
                std::array<int64_t, 3> t = {{cur, next, nextnext}};
                triangulated_faces.push_back(t);
                points_vector.erase(points_vector.begin() + next_i);
                break;
            } else {
                continue;
            }
        }

        if (no_colinear) break;
    }

    // cleanup convex polygon
    while (points_vector.size() >= 3) {
        std::array<int64_t, 3> t = {
            {points_vector[0], points_vector[1], points_vector[points_vector.size() - 1]}};
        triangulated_faces.push_back(t);
        points_vector.erase(points_vector.begin());
    }

    return triangulated_faces;
}

std::tuple<std::shared_ptr<wmtk::TetMesh>, std::vector<std::array<bool, 4>>>
generate_raw_tetmesh_from_input_surface(
    const RowVectors3d& V,
    const RowVectors3l& F,
    const double eps_target,
    const RowVectors3d& bgV,
    const RowVectors4l& bgT)
{
    // compute bounding box
    Vector3d bbox_min = V.colwise().minCoeff();
    Vector3d bbox_max = V.colwise().maxCoeff();
    double diag_length = (bbox_max - bbox_min).norm();
    double eps = 1 / 15.0; // TODO: set this flexible
    double delta = diag_length * eps;
    bbox_min -= Vector3d(delta, delta, delta);
    bbox_max += Vector3d(delta, delta, delta);
    double target_edge_length = diag_length * eps_target;

    // compute resolution
    std::array<int64_t, 3> resolution = {
        {(int64_t)((bbox_max[0] - bbox_min[0]) / target_edge_length),
         (int64_t)((bbox_max[1] - bbox_min[1]) / target_edge_length),
         (int64_t)((bbox_max[2] - bbox_min[2]) / target_edge_length)}};

    // generate background mesh
    RowVectors4l background_TV;
    RowVectors3d background_V;
    if (bgV.size() == 0) {
        generate_background_mesh(bbox_min, bbox_max, resolution, background_TV, background_V);

        wmtk::logger().info(
            "generated background mesh with resolution {} {} {}",
            resolution[0],
            resolution[1],
            resolution[2]);
    } else {
        assert(bgT.size() != 0);
        background_TV = bgT;
        background_V = bgV;
    }


    // prepare data for volume remesher
    // inputs
    std::vector<double> tri_vrt_coords(V.rows() * 3);
    std::vector<uint32_t> triangle_indices(F.rows() * 3);
    std::vector<double> tet_vrt_coords(background_V.rows() * 3);
    std::vector<uint32_t> tet_indices(background_TV.rows() * 4);

    for (int64_t i = 0; i < V.rows(); ++i) {
        tri_vrt_coords[i * 3 + 0] = V(i, 0);
        tri_vrt_coords[i * 3 + 1] = V(i, 1);
        tri_vrt_coords[i * 3 + 2] = V(i, 2);
    }

    for (int64_t i = 0; i < F.rows(); ++i) {
        triangle_indices[i * 3 + 0] = F(i, 0);
        triangle_indices[i * 3 + 1] = F(i, 1);
        triangle_indices[i * 3 + 2] = F(i, 2);
    }

    for (int64_t i = 0; i < background_V.rows(); ++i) {
        tet_vrt_coords[i * 3 + 0] = background_V(i, 0);
        tet_vrt_coords[i * 3 + 1] = background_V(i, 1);
        tet_vrt_coords[i * 3 + 2] = background_V(i, 2);
    }

    for (int64_t i = 0; i < background_TV.rows(); ++i) {
        tet_indices[i * 4 + 0] = background_TV(i, 0);
        tet_indices[i * 4 + 1] = background_TV(i, 1);
        tet_indices[i * 4 + 2] = background_TV(i, 2);
        tet_indices[i * 4 + 3] = background_TV(i, 3);
    }

    // outputs
    std::vector<vol_rem::bigrational> embedded_vertices;
    std::vector<uint32_t> embedded_facets;
    std::vector<uint32_t> embedded_cells;
    std::vector<uint32_t> embedded_facets_on_input;

    // run volume remesher
    vol_rem::embed_tri_in_poly_mesh(
        tri_vrt_coords,
        triangle_indices,
        tet_vrt_coords,
        tet_indices,
        embedded_vertices,
        embedded_facets,
        embedded_cells,
        embedded_facets_on_input,
        true);

    wmtk::logger().info(
        "volume remesher finished, polycell mesh generated, #vertices: {},  #cells: {}",
        embedded_vertices.size() / 3,
        embedded_cells.size());

    // convert to double and readable format
    // std::vector<Vector3d> v_coords; // vertex coordinates
    std::vector<Vector3r> v_coords; // vertex coordinates
    std::vector<std::vector<int64_t>> polygon_faces; // index of vertices
    std::vector<std::vector<int64_t>> polygon_cells; // index of faces
    std::vector<bool> polygon_faces_on_input; // whether the face is on input surface
    std::vector<std::array<int64_t, 4>> tets_final; // final tets
    std::vector<std::array<bool, 4>> tet_face_on_input_surface; // tet local face on input surface

    wmtk::logger().trace("Converting vertices...");
    for (int64_t i = 0; i < embedded_vertices.size() / 3; ++i) {
#ifdef USE_GNU_GMP_CLASSES
        v_coords.emplace_back();
        v_coords.back()[0].init(embedded_vertices[3 * i + 0].get_mpq_t());
        v_coords.back()[1].init(embedded_vertices[3 * i + 1].get_mpq_t());
        v_coords.back()[2].init(embedded_vertices[3 * i + 2].get_mpq_t());
#else
        v_coords.emplace_back();
        v_coords.back()[0].init_from_binary(embedded_vertices[3 * i + 0].get_str());
        v_coords.back()[1].init_from_binary(embedded_vertices[3 * i + 1].get_str());
        v_coords.back()[2].init_from_binary(embedded_vertices[3 * i + 2].get_str());
#endif
    }
    wmtk::logger().trace("done");

    wmtk::logger().trace("Facets loop...");
    for (int64_t i = 0; i < embedded_facets.size(); ++i) {
        int64_t polysize = embedded_facets[i];
        std::vector<int64_t> polygon;
        for (int64_t j = i + 1; j <= i + polysize; ++j) {
            polygon.push_back(embedded_facets[j]);
        }
        polygon_faces.push_back(polygon);
        i += polysize; // dangerous behavior, but working now
    }
    wmtk::logger().trace("done");

    wmtk::logger().trace("Cells loop...");
    for (int64_t i = 0; i < embedded_cells.size(); ++i) {
        std::vector<int64_t> polygon_cell;
        int64_t cellsize = embedded_cells[i];
        for (int64_t j = i + 1; j <= i + cellsize; ++j) {
            polygon_cell.push_back(embedded_cells[j]);
        }
        polygon_cells.push_back(polygon_cell);
        i += cellsize; // dangerous behavior, but working now
    }
    wmtk::logger().trace("done");

    polygon_faces_on_input.resize(polygon_faces.size(), false);

    for (int64_t i = 0; i < embedded_facets_on_input.size(); ++i) {
        polygon_faces_on_input[embedded_facets_on_input[i]] = true;
    }

    // triangulate polygon faces
    std::vector<std::array<int64_t, 3>> triangulated_faces;
    std::vector<bool> triangulated_faces_on_input;
    std::vector<std::vector<int64_t>> map_poly_to_tri_face(polygon_faces.size());

    wmtk::logger().trace("Triangulation loop...");
    for (int64_t i = 0; i < polygon_faces.size(); ++i) {
        // already clipped in other polygon
        if (map_poly_to_tri_face[i].size() != 0) continue;

        // new polygon face to clip
        std::vector<std::array<int64_t, 3>> clipped_indices;
        std::vector<Vector3r> poly_coordinates;
        const std::vector<int64_t>& polygon_face = polygon_faces[i];
        assert(polygon_face.size() >= 3);

        if (polygon_face.size() == 3) {
            // already a triangle, don't need to clip
            std::array<int64_t, 3> triangle_face = {
                {polygon_face[0], polygon_face[1], polygon_face[2]}};
            int64_t idx = triangulated_faces.size();
            triangulated_faces.push_back(triangle_face);
            if (polygon_faces_on_input[i]) {
                triangulated_faces_on_input.push_back(true);
            } else {
                triangulated_faces_on_input.push_back(false);
            }
            map_poly_to_tri_face[i].push_back(idx);
        } else {
            // not a triangle, need to clip
            for (int64_t j = 0; j < polygon_faces[i].size(); ++j) {
                poly_coordinates.push_back(v_coords[polygon_face[j]]);
            }

            // clip polygon face
            clipped_indices = triangulate_polygon_face(poly_coordinates);

            for (int64_t j = 0; j < clipped_indices.size(); ++j) {
                // need to map oldface index to new face indices
                std::array<int64_t, 3> triangle_face = {
                    {polygon_face[clipped_indices[j][0]],
                     polygon_face[clipped_indices[j][1]],
                     polygon_face[clipped_indices[j][2]]}};

                int64_t idx = triangulated_faces.size();
                triangulated_faces.push_back(triangle_face);

                // track input faces
                if (polygon_faces_on_input[i]) {
                    triangulated_faces_on_input.push_back(true);
                } else {
                    triangulated_faces_on_input.push_back(false);
                }
                map_poly_to_tri_face[i].push_back(idx);
            }
        }
    }

    wmtk::logger().info("triangulation finished.");

    // tetrahedralization polygon cells
    // int64_t was_tet_cnt = 0;
    for (int64_t i = 0; i < polygon_cells.size(); ++i) {
        auto polygon_cell = polygon_cells[i];

        // get polygon vertices
        std::vector<int64_t> polygon_vertices;
        for (auto f : polygon_cell) {
            for (auto v : polygon_faces[f]) {
                polygon_vertices.push_back(v);
            }
        }
        vector_unique(polygon_vertices);

        // compute number of triangle faces
        int64_t num_faces = 0;
        for (auto f : polygon_cell) {
            num_faces += map_poly_to_tri_face[f].size();
        }

        // polygon already a tet
        if (num_faces == 4) {
            // was_tet_cnt++;
            assert(polygon_vertices.size() == 4);
            // get the correct orientation here
            int64_t v0 = polygon_faces[polygon_cell[0]][0];
            int64_t v1 = polygon_faces[polygon_cell[0]][1];
            int64_t v2 = polygon_faces[polygon_cell[0]][2];
            int64_t v3 = -1;
            for (auto v : polygon_faces[polygon_cell[1]]) {
                if (v != v0 && v != v1 && v != v2) {
                    v3 = v;
                    break;
                }
            }

            assert(v3 != -1);

            std::array<int64_t, 4> tetra = {{v0, v1, v2, v3}};

            if (wmtk_orient3d(v_coords[v0], v_coords[v1], v_coords[v2], v_coords[v3]) < 0) {
                tetra = {{v1, v0, v2, v3}};
            }


            // push the tet to final queue;
            tets_final.push_back(tetra);

            std::set<int64_t> local_f0 = {tetra[1], tetra[2], tetra[3]};
            std::set<int64_t> local_f1 = {tetra[0], tetra[2], tetra[3]};
            std::set<int64_t> local_f2 = {tetra[0], tetra[1], tetra[3]};
            std::set<int64_t> local_f3 = {tetra[0], tetra[1], tetra[2]};

            // track surface
            std::array<bool, 4> tet_face_on_input;
            for (auto f : polygon_cell) {
                std::set<int64_t> f_vs = {
                    polygon_faces[f][0],
                    polygon_faces[f][1],
                    polygon_faces[f][2]};

                int64_t local_f_idx;

                // decide which face it is

                if (f_vs == local_f0) {
                    local_f_idx = 0;
                } else if (f_vs == local_f1) {
                    local_f_idx = 1;
                } else if (f_vs == local_f2) {
                    local_f_idx = 2;
                } else {
                    local_f_idx = 3;
                }

                tet_face_on_input[local_f_idx] = polygon_faces_on_input[f];
            }


            tet_face_on_input_surface.push_back(tet_face_on_input);

            continue;
        }

        // not a tet initially
        // compute centroid
        Vector3r centroid(0., 0., 0.);
        for (auto v : polygon_vertices) {
            centroid = centroid + v_coords[v];
        }
        centroid = centroid / double(polygon_vertices.size());

        // trahedralize
        int64_t centroid_idx = v_coords.size();
        v_coords.push_back(centroid);

        for (auto f : polygon_cell) {
            for (auto t : map_poly_to_tri_face[f]) {
                std::array<int64_t, 4> tetra = {
                    {triangulated_faces[t][0],
                     triangulated_faces[t][1],
                     triangulated_faces[t][2],
                     centroid_idx}};

                if (wmtk_orient3d(
                        v_coords[tetra[0]],
                        v_coords[tetra[1]],
                        v_coords[tetra[2]],
                        v_coords[tetra[3]]) < 0) {
                    tetra = {
                        {triangulated_faces[t][1],
                         triangulated_faces[t][0],
                         triangulated_faces[t][2],
                         centroid_idx}};
                }

                tets_final.push_back(tetra);


                tet_face_on_input_surface.push_back(
                    {{false, false, false, triangulated_faces_on_input[t]}});
            }
        }
    }

    wmtk::logger().info("tetrahedralization finished.");

    // remove unused vertices and map
    std::vector<bool> v_is_used_in_tet(v_coords.size(), false);
    for (const auto& t : tets_final) {
        for (const auto& v : t) {
            v_is_used_in_tet[v] = true;
        }
    }

    std::map<int64_t, int64_t> v_map;
    std::vector<Vector3d> v_coords_final;
    std::vector<Vector3r> v_coords_final_rational;

    for (int64_t i = 0; i < v_coords.size(); ++i) {
        if (v_is_used_in_tet[i]) {
            v_map[i] = v_coords_final.size();
            v_coords_final.emplace_back(
                v_coords[i][0].to_double(),
                v_coords[i][1].to_double(),
                v_coords[i][2].to_double());
            v_coords_final_rational.emplace_back(v_coords[i]);
        }
    }

    // remap tets
    for (auto& t : tets_final) {
        for (int i = 0; i < 4; ++i) {
            t[i] = v_map[t[i]];
        }

        if (wmtk_orient3d(
                v_coords_final_rational[t[0]],
                v_coords_final_rational[t[1]],
                v_coords_final_rational[t[2]],
                v_coords_final_rational[t[3]]) <= 0) {
            Eigen::Matrix<Rational, 3, 3> tmp;
            tmp.col(0) = v_coords_final_rational[t[1]] - v_coords_final_rational[t[0]];
            tmp.col(1) = v_coords_final_rational[t[2]] - v_coords_final_rational[t[0]];
            tmp.col(2) = v_coords_final_rational[t[3]] - v_coords_final_rational[t[0]];
            log_and_throw_error(
                "flipped tet=({},{},{},{}) crash vol={} orient={}",
                t[0],
                t[1],
                t[2],
                t[3],
                tmp.determinant().serialize(),
                wmtk_orient3d(
                    v_coords_final_rational[t[0]],
                    v_coords_final_rational[t[1]],
                    v_coords_final_rational[t[2]],
                    v_coords_final_rational[t[3]]));
        }
    }

    // transfer v_coords_final to V matrix and tets_final to TV matrix
    RowVectors3d V_final(v_coords_final.size(), 3);
    RowVectors3r V_final_rational(v_coords_final_rational.size(), 3);
    RowVectors4l TV_final(tets_final.size(), 4);


    for (int64_t i = 0; i < v_coords_final.size(); ++i) {
        V_final.row(i) = v_coords_final[i];
    }

    for (int64_t i = 0; i < v_coords_final_rational.size(); ++i) {
        V_final_rational.row(i) = v_coords_final_rational[i];
    }

    for (int64_t i = 0; i < tets_final.size(); ++i) {
        TV_final.row(i) << tets_final[i][0], tets_final[i][1], tets_final[i][2], tets_final[i][3];
    }

    wmtk::logger().info("remove unused vertices finished.");

    // initialize tetmesh
    std::shared_ptr<wmtk::TetMesh> m = std::make_shared<wmtk::TetMesh>();
    m->initialize(TV_final);
    // mesh_utils::set_matrix_attribute(V_final, "vertices", PrimitiveType::Vertex, *m);
    mesh_utils::set_matrix_attribute(V_final_rational, "vertices", PrimitiveType::Vertex, *m);

    wmtk::logger().info("init tetmesh finished.");

    if (!m->is_connectivity_valid()) {
        wmtk::logger().error("invalid tetmesh connectivity after insertion");
        throw std::runtime_error("invalid tetmesh connectivity by insertion");
    }

    return std::make_tuple(m, tet_face_on_input_surface);
}

std::tuple<std::shared_ptr<wmtk::TetMesh>, std::shared_ptr<wmtk::Mesh>>
generate_raw_tetmesh_with_surface_from_input(
    const RowVectors3d& V,
    const RowVectors3l& F,
    const double eps_target,
    const RowVectors3d& bgV,
    const RowVectors4l& bgF)
{
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    auto [tetmesh, tet_face_on_input_surface] =
        generate_raw_tetmesh_from_input_surface(V, F, eps_target, bgV, bgF);

    auto surface_handle =
        tetmesh->register_attribute<int64_t>("surface", PrimitiveType::Triangle, 1);
    auto surface_accessor = tetmesh->create_accessor<int64_t>(surface_handle);

    const auto& tets = tetmesh->get_all(PrimitiveType::Tetrahedron);
    assert(tets.size() == tet_face_on_input_surface.size());

    for (int64_t i = 0; i < tets.size(); ++i) {
        const auto& t = tets[i]; // local face 2
        std::array<Tuple, 4> fs = {
            {tetmesh->switch_tuples(t, {PV, PE, PF}),
             tetmesh->switch_tuples(t, {PE, PF}),
             t,
             tetmesh->switch_tuples(t, {PF})}};

        for (int64_t j = 0; j < 4; ++j) {
            if (tet_face_on_input_surface[i][j]) {
                surface_accessor.scalar_attribute(fs[j]) = 1;
            } else {
                surface_accessor.scalar_attribute(fs[j]) = 0;
            }
        }
    }

    auto child_ptr = wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
        *tetmesh,
        "surface",
        1,
        PF);

    wmtk::logger().info("registered surface child mesh to tetmesh. Tetmesh has face attribute with "
                        "name \"surface\"");

    return std::make_tuple(tetmesh, child_ptr);
}

} // namespace wmtk::utils
