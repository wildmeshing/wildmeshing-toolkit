
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

#include <polysolve/Utils.hpp>


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

void lookup_pentagon_table(
    const int pentagon_idx,
    const std::vector<int64_t>& face,
    std::vector<std::array<int64_t, 3>>& triangulated_faces)
{
    if (pentagon_idx < 10000) {
        // start with 0
        switch (pentagon_idx) {
        case 1111: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[2]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[2], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[3], face[4], face[0]}}));
            break;
        }
        case 1011: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[2]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[2], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[3], face[4], face[0]}}));
            break;
        }
        case 1101: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[2]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[2], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[3], face[4], face[0]}}));
            break;
        }
        case 111: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[2], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[3], face[4], face[0]}}));
            break;
        }
        case 1110: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[2]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[2], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[2], face[3], face[4]}}));
            break;
        }
        }
    } else {
        switch (pentagon_idx) {
        case 10111: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[2], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[3], face[4]}}));
            break;
        }
        case 11011: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[2]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[2], face[3], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[2], face[4], face[0]}}));
            break;
        }
        case 11101: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[2], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[3], face[4], face[0]}}));
            break;
        }
        case 11110: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[2], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[2], face[3], face[4]}}));
            break;
        }
        case 11010: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[2]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[2], face[3], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[4], face[0], face[2]}}));
            break;
        }
        case 10110: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[2], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[2], face[3], face[4]}}));
            break;
        }
        case 10101: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[2], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[3], face[4]}}));
            break;
        }
        case 10011: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[2], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[2], face[3], face[4]}}));
            break;
        }
        case 11001: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[2]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[2], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[3], face[4]}}));
            break;
        }
        case 11100: {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[2], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[3], face[4]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[4], face[0]}}));
            break;
        }
        }
    }
}

bool is_collinear(const Vector3r& a, const Vector3r& b, const Vector3r& c, const Vector3r& out)
{
    return wmtk_orient3d(a, b, c, out) == 0;
}

std::vector<std::array<int64_t, 3>> triangulate_polygon_face(
    const std::vector<Vector3r>& points,
    const std::vector<int64_t>& face)
{
    // triangulate weak convex polygons
    std::vector<std::array<int64_t, 3>> triangulated_faces;


    Vector3r out = points[0];
    for (int64_t i = 1; i < face.size(); ++i) {
        for (int64_t d = 0; d < 3; ++d) {
            if (out[d] < points[face[i]][d]) out[d] = points[face[i]][d];
        }
    }

    for (int64_t d = 0; d < 3; ++d) {
        out[d].round();
        out[d] = out[d] - 1;
        out[d].round();
    }

    if (face.size() == 4) {
        // special case for quad
        // if (((a[0] * b[1] - a[1] * b[0]).get_sign() != 0 ||
        //      (a[1] * b[2] - a[2] * b[1]).get_sign() != 0 ||
        //      (a[0] * b[2] - a[2] * b[0]).get_sign() != 0) &&
        //     ((c[0] * d[1] - c[1] * d[0]).get_sign() != 0 ||
        //      (c[1] * d[2] - c[2] * d[1]).get_sign() != 0 ||
        //      (c[0] * d[2] - c[2] * d[0]).get_sign() != 0)) {
        if (!is_collinear(points[face[0]], points[face[1]], points[face[3]], out) &&
            !is_collinear(points[face[1]], points[face[2]], points[face[3]], out)) {
            // not colinear continue
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[2], face[3]}}));
        } else {
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[2]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[2], face[3]}}));
        }

        return triangulated_faces;
    } else if (face.size() == 5) {
        // special case for pentagon
        // const Vector3r a = points[face[0]] - points[face[4]];
        // const Vector3r b = points[face[1]] - points[face[0]];
        // const Vector3r c = points[face[2]] - points[face[1]];
        // const Vector3r d = points[face[3]] - points[face[2]];
        // const Vector3r f = points[face[2]] - points[face[0]];
        // const Vector3r e = points[face[4]] - points[face[3]];

        bool noncolinear012 = !is_collinear(points[face[0]], points[face[1]], points[face[2]], out);
        //   (b[0] * c[1] - b[1] * c[0]).get_sign() != 0 ||
        //   (b[1] * c[2] - b[2] * c[1]).get_sign() != 0 ||
        //   (b[0] * c[2] - b[2] * c[0]).get_sign() != 0;

        bool noncolinear123 = !is_collinear(points[face[1]], points[face[2]], points[face[3]], out);
        //   (c[0] * d[1] - c[1] * d[0]).get_sign() != 0 ||
        //   (c[1] * d[2] - c[2] * d[1]).get_sign() != 0 ||
        //   (c[0] * d[2] - c[2] * d[0]).get_sign() != 0;

        bool noncolinear340 = !is_collinear(points[face[3]], points[face[4]], points[face[0]], out);
        //   (e[0] * a[1] - e[1] * a[0]).get_sign() != 0 ||
        //   (e[1] * a[2] - e[2] * a[1]).get_sign() != 0 ||
        //   (e[0] * a[2] - e[2] * a[0]).get_sign() != 0;

        // check bxc fxd exa
        if (noncolinear012 && noncolinear123 && noncolinear340) {
            // not colinear continue
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[0], face[1], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[1], face[2], face[3]}}));
            triangulated_faces.emplace_back(std::array<int64_t, 3>({{face[3], face[4], face[0]}}));

            return triangulated_faces;
        }

        // lookup table

        bool noncolinear023 = !is_collinear(points[face[0]], points[face[2]], points[face[3]], out);
        //   (f[0] * d[1] - f[1] * d[0]).get_sign() != 0 ||
        //   (f[1] * d[2] - f[2] * d[1]).get_sign() != 0 ||
        //   (f[0] * d[2] - f[2] * d[0]).get_sign() != 0;

        bool noncolinear234 = !is_collinear(points[face[2]], points[face[3]], points[face[4]], out);
        //   (d[0] * e[1] - d[1] * e[0]).get_sign() != 0 ||
        //   (d[1] * e[2] - d[2] * e[1]).get_sign() != 0 ||
        //   (d[0] * e[2] - d[2] * e[0]).get_sign() != 0;

        bool noncolinear401 = !is_collinear(points[face[4]], points[face[0]], points[face[1]], out);
        //   (a[0] * b[1] - a[1] * b[0]).get_sign() != 0 ||
        //   (a[1] * b[2] - a[2] * b[1]).get_sign() != 0 ||
        //   (a[0] * b[2] - a[2] * b[0]).get_sign() != 0;

        int pentagon_idx = 10000 * int(noncolinear401) + 1000 * int(noncolinear012) +
                           100 * int(noncolinear123) + 10 * int(noncolinear234) +
                           1 * int(noncolinear340);

        lookup_pentagon_table(pentagon_idx, face, triangulated_faces);

        return triangulated_faces;
    }

    // cache version
    std::vector<int64_t> points_vector = face;
    std::vector<bool> is_point_noncolinear(points_vector.size());

    // compute all angles
    int64_t colinear_cnt = 0;
    for (int64_t i = 0; i < points_vector.size(); ++i) {
        // const Vector3r a =
        //     points[points_vector[i]] -
        //     points[points_vector[(i + points_vector.size() - 1) % points_vector.size()]];

        // const Vector3r b =
        //     points[points_vector[(i + 1) % points_vector.size()]] - points[points_vector[i]];

        // if ((a[0] * b[1] - a[1] * b[0]).get_sign() != 0 ||
        //     (a[1] * b[2] - a[2] * b[1]).get_sign() != 0 ||
        //     (a[0] * b[2] - a[2] * b[0]).get_sign() != 0) {
        if (!is_collinear(
                points[points_vector[i]],
                points[points_vector[(i + points_vector.size() - 1) % points_vector.size()]],
                points[points_vector[(i + 1) % points_vector.size()]],
                out)) {
            is_point_noncolinear[i] = true;
        } else {
            is_point_noncolinear[i] = false;
            colinear_cnt++;
        }
    }

    // find the first colinear ABC with nonlinear BCD and delete C from vector
    while (colinear_cnt > 0) {
        for (int64_t i = 0; i < points_vector.size(); ++i) {
            if (!is_point_noncolinear[i] && is_point_noncolinear[(i + 1) % points_vector.size()]) {
                triangulated_faces.emplace_back(std::array<int64_t, 3>(
                    {{points_vector[i],
                      points_vector[(i + 1) % points_vector.size()],
                      points_vector[(i + 2) % points_vector.size()]}}));

                is_point_noncolinear[i] = true;
                points_vector.erase(points_vector.begin() + ((i + 1) % points_vector.size()));
                is_point_noncolinear.erase(
                    is_point_noncolinear.begin() + ((i + 1) % is_point_noncolinear.size()));
                break;
            }
        }
        colinear_cnt--;
    }

    assert(points_vector.size() >= 3);

    for (int64_t i = 1; i < points_vector.size() - 1; ++i) {
        triangulated_faces.emplace_back(
            std::array<int64_t, 3>({{points_vector[0], points_vector[i], points_vector[i + 1]}}));
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
    std::vector<std::array<uint32_t, 4>> tets_final; // final tets
    std::vector<uint32_t> final_tets_parent;
    std::vector<bool> cells_with_faces_on_input;
    std::vector<std::vector<uint32_t>> final_tets_parent_faces;

    // run volume remesher
    vol_rem::embed_tri_in_poly_mesh(
        tri_vrt_coords,
        triangle_indices,
        tet_vrt_coords,
        tet_indices,
        embedded_vertices,
        embedded_facets,
        embedded_cells,
        tets_final,
        final_tets_parent,
        embedded_facets_on_input,
        cells_with_faces_on_input,
        final_tets_parent_faces,
        wmtk::logger().level() < spdlog::level::info);

    assert(tets_final.size() == final_tets_parent.size());

    wmtk::logger().info(
        "volume remesher finished, polycell mesh generated, #vertices: {},  #cells: {}, #tets {}",
        embedded_vertices.size() / 3,
        embedded_cells.size(),
        tets_final.size());

    // convert to double and readable format
    std::vector<Vector3r> v_coords; // vertex coordinates
    std::vector<std::array<int64_t, 3>> polygon_faces; // index of vertices
    std::vector<bool> polygon_faces_on_input; // whether the face is on input surface
    std::vector<std::array<bool, 4>> tet_face_on_input_surface; // tet local face on input surface


    wmtk::logger().info("Converting vertices...");
    int64_t round_cnt = 0;
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


        bool round = true;
        for (int64_t j = 0; j < 3; ++j) {
            if (!v_coords.back()[j].can_be_rounded()) {
                round = false;
                break;
            }
        }

        if (round) {
            round_cnt++;
            for (int64_t j = 0; j < 3; ++j) v_coords.back()[j].round();
        }
    }
    wmtk::logger().info("done, {} rounded vertices over {}", round_cnt, v_coords.size());

    wmtk::logger().info("Facets loop...");
    for (int64_t i = 0; i < embedded_facets.size(); ++i) {
        int64_t polysize = embedded_facets[i];
        assert(polysize == 3);
        std::array<int64_t, 3> polygon;
        for (int64_t j = i + 1; j <= i + polysize; ++j) {
            polygon[j - (i + 1)] = embedded_facets[j];
        }
        polygon_faces.push_back(polygon);
        i += polysize; // dangerous behavior, but working now
    }
    wmtk::logger().info("done");

    wmtk::logger().info("Tags loop...");
    polygon_faces_on_input.resize(polygon_faces.size(), false);
    for (int64_t i = 0; i < embedded_facets_on_input.size(); ++i) {
        polygon_faces_on_input[embedded_facets_on_input[i]] = true;
    }
    wmtk::logger().info("done");


    wmtk::logger().info("tracking surface starting...");
    polysolve::StopWatch timer("tracking surface starting", logger());
    timer.start();
    assert(final_tets_parent_faces.size() == tets_final.size());
    for (int64_t i = 0; i < tets_final.size(); ++i) {
        const auto& tetra = tets_final[i];
        const uint32_t tetra_parent = final_tets_parent[i];

        if (!cells_with_faces_on_input[tetra_parent]) {
            tet_face_on_input_surface.push_back({false, false, false, false});
            continue;
        }

        // vector of std array and sort
        std::set<int64_t> local_f0 = {tetra[1], tetra[2], tetra[3]};
        std::set<int64_t> local_f1 = {tetra[0], tetra[2], tetra[3]};
        std::set<int64_t> local_f2 = {tetra[0], tetra[1], tetra[3]};
        std::set<int64_t> local_f3 = {tetra[0], tetra[1], tetra[2]};

        // track surface
        std::array<bool, 4> tet_face_on_input{{false, false, false, false}};
        for (auto f : final_tets_parent_faces[i]) {
            assert(polygon_faces[f].size() == 3);

            std::set<int64_t> f_vs = {
                polygon_faces[f][0],
                polygon_faces[f][1],
                polygon_faces[f][2]};

            int64_t local_f_idx = -1;

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
            assert(local_f_idx >= 0);

            tet_face_on_input[local_f_idx] = polygon_faces_on_input[f];
        }

        tet_face_on_input_surface.push_back(tet_face_on_input);
    }
    timer.stop();
    wmtk::logger().info("took: {}", timer.getElapsedTimeInSec());

    wmtk::logger().info("removing unreferenced vertices...");
    timer.start();
    std::vector<bool> v_is_used_in_tet(v_coords.size(), false);
    for (const auto& t : tets_final) {
        for (const auto& v : t) {
            v_is_used_in_tet[v] = true;
        }
    }
    std::vector<int64_t> v_map(v_coords.size(), -1);
    std::vector<Vector3r> v_coords_final;

    for (int64_t i = 0; i < v_coords.size(); ++i) {
        if (v_is_used_in_tet[i]) {
            v_map[i] = v_coords_final.size();
            v_coords_final.emplace_back(v_coords[i]);
        }
    }
    for (auto& t : tets_final) {
        for (int i = 0; i < 4; ++i) {
            assert(v_map[t[i]] >= 0);
            t[i] = v_map[t[i]];
        }
    }
    timer.stop();
    wmtk::logger().info("took: {}", timer.getElapsedTimeInSec());


    // check tets
    // for (auto& t : tets_final) {
    //     if (wmtk_orient3d(
    //             v_coords_final_rational[t[1]],
    //             v_coords_final_rational[t[0]],
    //             v_coords_final_rational[t[2]],
    //             v_coords_final_rational[t[3]]) <= 0) {
    //         Eigen::Matrix<Rational, 3, 3> tmp;
    //         tmp.col(0) = v_coords_final_rational[t[1]] - v_coords_final_rational[t[0]];
    //         tmp.col(1) = v_coords_final_rational[t[2]] - v_coords_final_rational[t[0]];
    //         tmp.col(2) = v_coords_final_rational[t[3]] - v_coords_final_rational[t[0]];
    //         log_and_throw_error(
    //             "flipped tet=({},{},{},{}) crash vol={} orient={}",
    //             t[0],
    //             t[1],
    //             t[2],
    //             t[3],
    //             tmp.determinant().serialize(),
    //             wmtk_orient3d(
    //                 v_coords_final_rational[t[1]],
    //                 v_coords_final_rational[t[0]],
    //                 v_coords_final_rational[t[2]],
    //                 v_coords_final_rational[t[3]]));
    //     }
    // }

    wmtk::logger().info("convert to eigen...");
    // transfer v_coords_final to V matrix and tets_final to TV matrix
    RowVectors3r V_final_rational(v_coords_final.size(), 3);
    RowVectors4l TV_final(tets_final.size(), 4);

    for (int64_t i = 0; i < v_coords_final.size(); ++i) {
        V_final_rational.row(i) = v_coords_final[i];
    }

    for (int64_t i = 0; i < tets_final.size(); ++i) {
        assert(tets_final[i][0] >= 0);
        assert(tets_final[i][1] >= 0);
        assert(tets_final[i][2] >= 0);
        assert(tets_final[i][3] >= 0);
        assert(tets_final[i][0] < V_final_rational.rows());
        assert(tets_final[i][1] < V_final_rational.rows());
        assert(tets_final[i][2] < V_final_rational.rows());
        assert(tets_final[i][3] < V_final_rational.rows());

        assert(tets_final[i][0] != tets_final[i][1]);
        assert(tets_final[i][0] != tets_final[i][2]);
        assert(tets_final[i][0] != tets_final[i][3]);

        assert(tets_final[i][1] != tets_final[i][2]);
        assert(tets_final[i][1] != tets_final[i][3]);

        assert(tets_final[i][2] != tets_final[i][3]);

        TV_final.row(i) << tets_final[i][0], tets_final[i][1], tets_final[i][2], tets_final[i][3];
    }
    wmtk::logger().info("done");

    wmtk::logger().info("init tetmesh...");
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
