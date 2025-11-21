#include "EmbedSurface.hpp"

// clang-format off
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/remove_unreferenced.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
// igl must be included BEFORE VolumeRemesher
#include <VolumeRemesher/embed.h>
#include <VolumeRemesher/numerics.h>
// clang-format on

#include <wmtk/utils/VectorUtils.h>
#include <bitset>
#include <filesystem>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/utils/InsertTriangleUtils.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/Reader.hpp>
#include <wmtk/utils/io.hpp>

#include <sec/ShortestEdgeCollapse.h>

#include "extract_soup.hpp"

namespace {
std::vector<std::array<size_t, 3>> triangulate_polygon_face(std::vector<wmtk::Vector3r> points)
{
    using namespace wmtk;

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

        if (no_colinear) {
            break;
        }
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
} // namespace

namespace wmtk::components::image_simulation {

void delaunay_box_mesh(
    const wmtk::SampleEnvelope& envelope,
    const MatrixXd& vertices,
    std::vector<wmtk::delaunay::Point3D>& points,
    std::vector<wmtk::delaunay::Tetrahedron>& tets,
    Vector3d& box_min,
    Vector3d& box_max)
{
    assert(vertices.cols() == 3);

    const Vector3d vertices_max = vertices.colwise().maxCoeff();
    const Vector3d vertices_min = vertices.colwise().minCoeff();
    const double diag = (vertices_max - vertices_min).norm();

    ///points for delaunay
    points.resize(vertices.rows());
    for (int i = 0; i < vertices.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            points[i][j] = vertices(i, j);
        }
    }
    // bbox
    double delta = diag / 15.0;
    box_min = Vector3d(vertices_min[0] - delta, vertices_min[1] - delta, vertices_min[2] - delta);
    box_max = Vector3d(vertices_max[0] + delta, vertices_max[1] + delta, vertices_max[2] + delta);

    // add corners of domain
    for (int i = 0; i < 8; i++) {
        Vector3d p;
        std::bitset<sizeof(int) * 8> a(i);
        for (int j = 0; j < 3; j++) {
            if (a.test(j)) {
                p[j] = box_max[j];
            } else {
                p[j] = box_min[j];
            }
        }
        points.push_back({{p[0], p[1], p[2]}});
    }

    const double voxel_resolution = diag / 20.0;
    std::array<int, 3> N; // number of grid points per dimension
    std::array<double, 3> h; // distance between grid points per dimension
    for (int i = 0; i < 3; i++) {
        const double D = box_max[i] - box_min[i];
        N[i] = (D / voxel_resolution) + 1;
        h[i] = D / N[i];
    }

    std::array<std::vector<double>, 3> ds;
    for (int i = 0; i < 3; i++) {
        ds[i].push_back(box_min[i]);
        for (int j = 0; j < N[i] - 1; j++) {
            ds[i].push_back(box_min[i] + h[i] * (j + 1));
        }
        ds[i].push_back(box_max[i]);
    }

    const double min_dis = voxel_resolution * voxel_resolution / 4;
    //    double min_dis = state.target_edge_len * state.target_edge_len;//epsilon*2
    for (int i = 0; i < ds[0].size(); i++) {
        for (int j = 0; j < ds[1].size(); j++) {
            for (int k = 0; k < ds[2].size(); k++) {
                if ((i == 0 || i == ds[0].size() - 1) && (j == 0 || j == ds[1].size() - 1) &&
                    (k == 0 || k == ds[2].size() - 1)) {
                    continue;
                }
                const Vector3d p(ds[0][i], ds[1][j], ds[2][k]);

                Eigen::Vector3d n;
                const double sqd = envelope.nearest_point(p, n);

                if (sqd < min_dis) {
                    continue;
                }
                points.push_back({{ds[0][i], ds[1][j], ds[2][k]}});
            }
        }
    }

    ///delaunay
    std::vector<wmtk::delaunay::Point3D> unused_points;
    std::tie(unused_points, tets) = wmtk::delaunay::delaunay3D(points);
    wmtk::logger().info(
        "after delauney tets.size() = {} | points.size() = {}",
        tets.size(),
        points.size());
}

void embed_surface(
    const MatrixXd& V_surface,
    const MatrixXi& F_surface,
    const MatrixXd& V_vol,
    const MatrixXi& T_vol,
    MatrixXr& V_emb,
    MatrixXi& T_emb)
{
    MatrixXi F_on_surface;
    embed_surface(V_surface, F_surface, V_vol, T_vol, V_emb, T_emb, F_on_surface);
}

void embed_surface(
    const MatrixXd& V_surface,
    const MatrixXi& F_surface,
    const MatrixXd& V_vol,
    const MatrixXi& T_vol,
    MatrixXr& V_emb,
    MatrixXi& T_emb,
    MatrixXi& F_on_surface)
{
    // old output
    std::vector<std::array<size_t, 3>> facets_after;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets_after;
    std::vector<bool> tet_face_on_input_surface;
    std::vector<Vector3r> v_rational;

    ////
    std::vector<double> tet_ver_coord(V_vol.size());
    std::vector<uint32_t> tet_index(T_vol.size());
    std::cout << "tetver size: " << V_vol.rows() << std::endl;
    std::cout << "tet size: " << T_vol.rows() << std::endl;

    for (int i = 0; i < V_vol.rows(); ++i) {
        tet_ver_coord[3 * i + 0] = V_vol(i, 0);
        tet_ver_coord[3 * i + 1] = V_vol(i, 1);
        tet_ver_coord[3 * i + 2] = V_vol(i, 2);
    }

    for (int i = 0; i < T_vol.rows(); ++i) {
        tet_index[4 * i + 0] = T_vol(i, 0);
        tet_index[4 * i + 1] = T_vol(i, 1);
        tet_index[4 * i + 2] = T_vol(i, 2);
        tet_index[4 * i + 3] = T_vol(i, 3);
    }

    // prepare input surfaces info
    std::vector<double> tri_ver_coord(V_surface.size());
    std::vector<uint32_t> tri_index(F_surface.size());

    for (int i = 0; i < V_surface.rows(); ++i) {
        tri_ver_coord[3 * i + 0] = V_surface(i, 0);
        tri_ver_coord[3 * i + 1] = V_surface(i, 1);
        tri_ver_coord[3 * i + 2] = V_surface(i, 2);
    }

    for (int i = 0; i < F_surface.rows(); ++i) {
        tri_index[3 * i + 0] = F_surface(i, 0);
        tri_index[3 * i + 1] = F_surface(i, 1);
        tri_index[3 * i + 2] = F_surface(i, 2);
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

    assert(embedded_vertices.size() % 3 == 0);
    v_rational.resize(embedded_vertices.size() / 3);
    for (int i = 0; i < embedded_vertices.size() / 3; i++) {
#ifdef USE_GNU_GMP_CLASSES
        v_rational[i][0].init(embedded_vertices[3 * i + 0].get_mpq_t());
        v_rational[i][1].init(embedded_vertices[3 * i + 1].get_mpq_t());
        v_rational[i][2].init(embedded_vertices[3 * i + 2].get_mpq_t());
#else
        v_rational[i][0].init_from_bin(embedded_vertices[3 * i + 0].get_str());
        v_rational[i][1].init_from_bin(embedded_vertices[3 * i + 1].get_str());
        v_rational[i][2].init_from_bin(embedded_vertices[3 * i + 2].get_str());
#endif
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
        if (map_poly_to_tri_face[i].size() != 0) {
            continue;
        }

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
            for (const size_t v : polygon_faces[polygon_cell[1]]) {
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
            std::array<bool, 4> tet_face_on_input;
            for (const size_t f : polygon_cell) {
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

        for (const size_t f : polygon_cell) {
            for (const size_t t : map_poly_to_tri_face[f]) {
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

    T_emb.resize(tets_final.size(), 4);
    for (int i = 0; i < tets_final.size(); ++i) {
        const auto& t = tets_final[i];
        T_emb.row(i) = Vector4i(t[0], t[1], t[2], t[3]);
    }

    V_emb.resize(v_rational.size(), 3);
    for (int i = 0; i < v_rational.size(); ++i) {
        V_emb.row(i) = v_rational[i];
    }

    std::set<size_t> v_on_input;
    for (int i = 0; i < triangulated_faces.size(); ++i) {
        if (!triangulated_faces_on_input[i]) {
            continue;
        }
        const auto& f = triangulated_faces[i];
        v_on_input.insert(f[0]);
        v_on_input.insert(f[1]);
        v_on_input.insert(f[2]);
    }

    size_t n_face_on_input = 0;
    for (const bool b : triangulated_faces_on_input) {
        if (b) {
            ++n_face_on_input;
        }
    }

    F_on_surface.resize(n_face_on_input, 3);
    n_face_on_input = 0;
    for (int i = 0; i < triangulated_faces.size(); ++i) {
        if (!triangulated_faces_on_input[i]) {
            continue;
        }
        const auto& f = triangulated_faces[i];
        F_on_surface.row(n_face_on_input++) = Vector3i(f[0], f[1], f[2]);
    }


    std::cout << "v on surface vector size: " << v_rational.size() << std::endl;
    std::cout << "v on surface: " << v_on_input.size() << std::endl;
}

void tag_tets_from_image(
    const std::string& filename,
    const Matrix4d& xyz2ijk,
    const MatrixXd& V,
    const MatrixXi& T,
    VectorXi& T_tags)

{
    logger().info("Tag tets");
    T_tags.resize(T.rows(), 1);
    T_tags.setZero();

    std::vector<std::vector<std::vector<size_t>>> volumetric_data;
    read_array_data_ascii(volumetric_data, filename);

    tag_tets_from_image(volumetric_data, xyz2ijk, V, T, T_tags);
}

void tag_tets_from_image(
    const ImageData& data,
    const Matrix4d& xyz2ijk,
    const MatrixXd& V,
    const MatrixXi& T,
    VectorXi& T_tags)
{
    T_tags.resize(T.rows(), 1);
    T_tags.setZero();

    for (size_t i = 0; i < T.rows(); ++i) {
        const Vector3d v0 = V.row(T(i, 0));
        const Vector3d v1 = V.row(T(i, 1));
        const Vector3d v2 = V.row(T(i, 2));
        const Vector3d v3 = V.row(T(i, 3));

        Vector3d center = (v0 + v1 + v2 + v3) * 0.25;
        // map from geometric position to index space, i.e., undo dimension multiplication
        //center[0] /= dimensions[0];
        //center[1] /= dimensions[1];
        //center[2] /= dimensions[2];
        {
            Vector4d ch = to_homogenuous(center);
            ch = xyz2ijk * ch;
            center = from_homogenuous(ch);
        }
        const int idx_0 = std::floor(center.x());
        const int idx_1 = std::floor(center.y());
        const int idx_2 = std::floor(center.z());
        // mesh.m_tet_attribute[t.tid(mesh)].tag = idx_0;
        if (idx_0 >= 0 && idx_0 < data.size() && idx_1 >= 0 && idx_1 < data[0].size() &&
            idx_2 >= 0 && idx_2 < data[0][0].size()) {
            // for tag
            int64_t intValue = data[idx_0][idx_1][idx_2];
            T_tags[i] = intValue;
        }
    }
}

void tag_tets_from_images(
    const std::vector<ImageData>& data,
    const Matrix4d& xyz2ijk,
    const MatrixXd& V,
    const MatrixXi& T,
    MatrixXi& T_tags)
{
    T_tags.resize(T.rows(), data.size());
    T_tags.setZero();

    for (size_t i = 0; i < data.size(); ++i) {
        VectorXi t;
        tag_tets_from_image(data[i], xyz2ijk, V, T, t);
        T_tags.col(i) = t;
    }
}

EmbedSurface::EmbedSurface(const std::vector<std::string>& img_filenames, const Matrix4d& ijk2xyz)
    : m_img_filenames(img_filenames)
    , m_ijk2xyz(ijk2xyz)
    , m_xyz2ijk(ijk2xyz.inverse())
{
    m_img_datas.resize(m_img_filenames.size());
    for (size_t i = 0; i < m_img_filenames.size(); ++i) {
        if (!std::filesystem::exists(m_img_filenames[i])) {
            log_and_throw_error("Image {} does not exist", m_img_filenames[i]);
        }
        read_array_data_ascii(m_img_datas[i], m_img_filenames[i]);
        MatrixXd V_single;
        MatrixXi F_single;
        extract_triangle_soup_from_image(m_img_datas[i], F_single, V_single);

        assert(V_single.cols() == 3);
        assert(F_single.cols() == 3);

        const size_t nV_old = m_V_surface.rows();
        const size_t nF_old = m_F_surface.rows();

        m_V_surface.conservativeResize(m_V_surface.rows() + V_single.rows(), 3);
        m_V_surface.block(nV_old, 0, V_single.rows(), 3) = V_single;

        F_single.array() += nV_old;
        m_F_surface.conservativeResize(m_F_surface.rows() + F_single.rows(), 3);
        m_F_surface.block(nF_old, 0, F_single.rows(), 3) = F_single;
    }


    // process triangle soup

    MatrixXd V;
    MatrixXi F;
    VectorXi _I;

    igl::remove_unreferenced(m_V_surface, m_F_surface, V, F, _I);

    if (V.rows() == 0 || F.rows() == 0) {
        log_and_throw_error("Empty Input");
    }

    const std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax =
        std::pair(V.colwise().minCoeff(), V.colwise().maxCoeff());
    double diag = (box_minmax.first - box_minmax.second).norm();

    // using the same error tolerance as in tetwild
    Eigen::VectorXi SVI, SVJ, SVK;
    Eigen::MatrixXd temp_V = V; // for STL file

    const Vector4d single_voxel_max = ijk2xyz * Vector4d::Ones();
    const Vector4d single_voxel_min = ijk2xyz * Vector4d(0, 0, 0, 1);
    double eps = (from_homogenuous(single_voxel_max) - from_homogenuous(single_voxel_min))
                     .cwiseAbs()
                     .minCoeff() *
                 0.01;
    if (eps <= 0) {
        logger().warn("EPS = {}, ijk_to_ras matix might be broken! Changing eps to 1e-4", eps);
        eps = 1e-4;
    }

    igl::remove_duplicate_vertices(temp_V, eps, V, SVI, SVJ);
    for (int i = 0; i < F.rows(); i++) {
        for (int j : {0, 1, 2}) {
            F(i, j) = SVJ[F(i, j)];
        }
    }
    MatrixXi F1 = F;

    resolve_duplicated_faces(F1, F);

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    verts.resize(V.rows());
    tris.resize(F.rows());
    wmtk::eigen_to_wmtk_input(verts, tris, V, F);

    wmtk::logger().info("after remove duplicate v#: {} f#: {}", V.rows(), F.rows());

    Eigen::VectorXi dummy;

    if (!igl::is_edge_manifold(F) || !igl::is_vertex_manifold(F, dummy)) {
        auto v1 = verts;
        auto tri1 = tris;
        logger().info("Separate to manifold");
        wmtk::separate_to_manifold(v1, tri1, verts, tris, modified_nonmanifold_v);
    }

    // apply dimensions
    logger().info("Convert from image to world coordinates (ijk to xyz)");
    for (Vector3d& v : verts) {
        Vector4d vh = to_homogenuous(v);
        vh = m_ijk2xyz * vh;
        v = from_homogenuous(vh);
    }
    logger().info("Finished conversion.");

    V_surf_from_vector(verts);
    F_surf_from_vector(tris);
}

void EmbedSurface::simplify_surface(const double eps)
{
    // convert to STL vectors
    std::vector<Eigen::Vector3d> verts = V_surf_to_vector();
    std::vector<std::array<size_t, 3>> tris = F_surf_to_vector();

    app::sec::ShortestEdgeCollapse surf_mesh(verts, 0, false);

    // must be a small envelope to ensure correct tet tags later on
    surf_mesh.create_mesh(verts.size(), tris, modified_nonmanifold_v, eps);
    assert(surf_mesh.check_mesh_connectivity_validity());

    surf_mesh.collapse_shortest(0);
    surf_mesh.consolidate_mesh();
    // surf_mesh.write_triangle_mesh("triangle_soup_coarse.off");

    //// get the simplified input
    std::vector<Eigen::Vector3d> v_simplified;
    std::vector<std::array<size_t, 3>> f_simplified;
    v_simplified.resize(surf_mesh.vert_capacity());
    f_simplified.resize(surf_mesh.tri_capacity());
    for (const auto& t : surf_mesh.get_vertices()) {
        const size_t i = t.vid(surf_mesh);
        v_simplified[i] = surf_mesh.vertex_attrs[i].pos;
    }

    for (const auto& t : surf_mesh.get_faces()) {
        const auto i = t.fid(surf_mesh);
        const auto vs = surf_mesh.oriented_tri_vids(t);
        for (int j = 0; j < 3; j++) {
            f_simplified[i][j] = vs[j];
        }
    }

    V_surf_from_vector(v_simplified);
    F_surf_from_vector(f_simplified);
}

void EmbedSurface::remove_duplicates(const double eps)
{
    auto v = V_surf_to_vector();
    auto f = F_surf_to_vector();

    wmtk::remove_duplicates(v, f, eps);

    V_surf_from_vector(v);
    F_surf_from_vector(f);
}

bool EmbedSurface::embed_surface()
{
    logger().info("Embed with VolumeInsertion");

    std::shared_ptr<SampleEnvelope> ptr_env;
    {
        const auto v_simplified = V_surf_to_vector();

        std::vector<Eigen::Vector3i> tempF(m_F_surface.rows());
        for (size_t i = 0; i < tempF.size(); ++i) {
            tempF[i] = m_F_surface.row(i);
        }
        ptr_env = std::make_shared<SampleEnvelope>();
        ptr_env->init(v_simplified, tempF, 0.5);
    }

    std::vector<wmtk::delaunay::Point3D> points;
    std::vector<wmtk::delaunay::Tetrahedron> tets;
    Vector3d box_min;
    Vector3d box_max;
    delaunay_box_mesh(*ptr_env, m_V_surface, points, tets, box_min, box_max);

    MatrixXd V_vol;
    V_vol.resize(points.size(), 3);
    for (int i = 0; i < points.size(); ++i) {
        const auto& v = points[i];
        V_vol.row(i) = Vector3d(v[0], v[1], v[2]);
    }

    MatrixXi T_vol;
    T_vol.resize(tets.size(), 4);
    for (int i = 0; i < tets.size(); ++i) {
        const auto& t = tets[i];
        T_vol.row(i) = Vector4i(t[0], t[1], t[2], t[3]);
    }

    wmtk::components::image_simulation::embed_surface(
        m_V_surface,
        m_F_surface,
        V_vol,
        T_vol,
        m_V_emb_r,
        m_T_emb,
        m_F_on_surface);

    const bool all_rounded = VF_rational_to_double(m_V_emb_r, m_T_emb, m_V_emb);

    if (!all_rounded) {
        // log_and_throw_error("Tets are inverted after converting to double precision.");
        logger().info("Not all vertices can be rounded to double precision.");
    }

    // add tags
    tag_tets_from_images(m_img_datas, m_xyz2ijk, m_V_emb, m_T_emb, m_T_tags);

    return all_rounded;
}

bool EmbedSurface::embed_surface_tetgen()
{
    logger().info("Embed with TetGen");

    std::shared_ptr<SampleEnvelope> ptr_env;
    {
        const auto v_simplified = V_surf_to_vector();

        std::vector<Eigen::Vector3i> tempF(m_F_surface.rows());
        for (size_t i = 0; i < tempF.size(); ++i) {
            tempF[i] = m_F_surface.row(i);
        }
        ptr_env = std::make_shared<SampleEnvelope>();
        ptr_env->init(v_simplified, tempF, 0.5);
    }

    Eigen::MatrixXd V_in;
    Eigen::MatrixXi F_in;
    {
        const Vector3d vertices_max = m_V_surface.colwise().maxCoeff();
        const Vector3d vertices_min = m_V_surface.colwise().minCoeff();
        const double diag = (vertices_max - vertices_min).norm();

        // bbox
        double delta = diag / 15.0;
        const Vector3d box_min(
            vertices_min[0] - delta,
            vertices_min[1] - delta,
            vertices_min[2] - delta);
        const Vector3d box_max(
            vertices_max[0] + delta,
            vertices_max[1] + delta,
            vertices_max[2] + delta);


        // add corners of domain
        std::vector<Vector3d> points(8);
        for (int i = 0; i < 8; i++) {
            Vector3d& p = points[i];
            std::bitset<sizeof(int) * 8> a(i);
            for (int j = 0; j < 3; j++) {
                if (a.test(j)) {
                    p[j] = box_max[j];
                } else {
                    p[j] = box_min[j];
                }
            }
        }

        //const double voxel_resolution = diag / 20.0;
        //std::array<int, 3> N; // number of grid points per dimension
        //std::array<double, 3> h; // distance between grid points per dimension
        // for (int i = 0; i < 3; i++) {
        //     const double D = box_max[i] - box_min[i];
        //    N[i] = (D / voxel_resolution) + 1;
        //    h[i] = D / N[i];
        //}
        //
        // std::array<std::vector<double>, 3> ds;
        // for (int i = 0; i < 3; i++) {
        //    ds[i].push_back(box_min[i]);
        //    for (int j = 0; j < N[i] - 1; j++) {
        //        ds[i].push_back(box_min[i] + h[i] * (j + 1));
        //    }
        //    ds[i].push_back(box_max[i]);
        //}
        //
        //const double min_dis = voxel_resolution * voxel_resolution / 4;
        ////    double min_dis = state.target_edge_len * state.target_edge_len;//epsilon*2
        // for (int i = 0; i < ds[0].size(); i++) {
        //     for (int j = 0; j < ds[1].size(); j++) {
        //         for (int k = 0; k < ds[2].size(); k++) {
        //             if ((i == 0 || i == ds[0].size() - 1) && (j == 0 || j == ds[1].size() - 1) &&
        //                 (k == 0 || k == ds[2].size() - 1)) {
        //                 continue;
        //             }
        //             const Vector3d p(ds[0][i], ds[1][j], ds[2][k]);
        //
        //             Eigen::Vector3d n;
        //             const double sqd = ptr_env->nearest_point(p, n);
        //
        //             if (sqd < min_dis) {
        //                 continue;
        //             }
        //             points.emplace_back(ds[0][i], ds[1][j], ds[2][k]);
        //         }
        //     }
        // }


        V_in.resize(m_V_surface.rows() + points.size(), 3);
        V_in.block(0, 0, m_V_surface.rows(), 3) = m_V_surface;
        for (size_t i = 0; i < points.size(); ++i) {
            V_in.row(m_V_surface.rows() + i) = points[i];
        }

        F_in = m_F_surface;
    }

    MatrixXi TF;

    int ret = igl::copyleft::tetgen::tetrahedralize(V_in, F_in, "pq1.414Yc", m_V_emb, m_T_emb, TF);
    if (ret != 0) {
        log_and_throw_error("Tetwild returned with {}", ret);
    }

    m_V_emb_r.resizeLike(m_V_emb);
    for (int i = 0; i < m_V_emb.rows(); ++i) {
        m_V_emb_r.row(i) = to_rational(m_V_emb.row(i));
    }

    // add tags
    tag_tets_from_images(m_img_datas, m_xyz2ijk, m_V_emb, m_T_emb, m_T_tags);

    return true;
}

void EmbedSurface::consolidate()
{
    std::map<size_t, size_t> old2new;
    std::map<size_t, size_t> new2old;
    size_t new_vid_counter = 0;
    for (size_t i = 0; i < m_T_emb.rows(); ++i) {
        for (size_t j = 0; j < 4; ++j) {
            const auto vid = m_T_emb(i, j);
            if (old2new.count(vid) == 0) {
                old2new[vid] = new_vid_counter;
                new2old[new_vid_counter] = vid;
                ++new_vid_counter;
            }
        }
    }

    MatrixXd V;
    MatrixXr Vr;
    V.resize(new_vid_counter, 3);
    Vr.resize(new_vid_counter, 3);
    for (size_t i = 0; i < new_vid_counter; ++i) {
        V.row(i) = m_V_emb.row(new2old[i]);
        Vr.row(i) = m_V_emb_r.row(new2old[i]);
    }

    MatrixXi T;
    T.resizeLike(m_T_emb);
    for (size_t i = 0; i < m_T_emb.rows(); ++i) {
        for (size_t j = 0; j < 4; ++j) {
            T(i, j) = old2new[m_T_emb(i, j)];
        }
    }

    MatrixXi F_surf;
    F_surf.resizeLike(m_F_on_surface);
    for (size_t i = 0; i < m_F_on_surface.rows(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            F_surf(i, j) = old2new[m_F_on_surface(i, j)];
        }
    }

    m_V_emb = V;
    m_V_emb_r = Vr;
    m_T_emb = T;
    m_F_on_surface = F_surf;
}

void EmbedSurface::write_surf_off(const std::string& filename) const
{
    igl::writeOFF(filename, m_V_surface, m_F_surface);
}

void EmbedSurface::write_emb_surf_off(const std::string& filename) const
{
    igl::writeOFF(filename, m_V_emb, m_F_on_surface);
}

void EmbedSurface::write_emb_msh(const std::string& filename) const
{
    wmtk::MshData msh;
    msh.add_tet_vertices(m_V_emb.rows(), [this](size_t k) -> Vector3d { return m_V_emb.row(k); });

    const size_t n_tet_vertices = m_V_emb.rows();

    msh.add_tets(m_T_emb.rows(), [this](size_t k) { return m_T_emb.row(k); });

    for (size_t i = 0; i < m_T_tags.cols(); ++i) {
        msh.add_tet_attribute<1>(fmt::format("tag_{}", i), [this, i](size_t j) {
            return m_T_tags(j, i);
        });
    }

    msh.add_physical_group("ImageVolume");

    msh.add_face_vertices();
    msh.add_faces(m_F_on_surface.rows(), [this](size_t k) { return m_F_on_surface.row(k); });
    msh.add_physical_group("EmbeddedSurface");

    msh.add_face_vertices(m_V_surface.rows(), [this](size_t k) { return m_V_surface.row(k); });
    msh.add_faces(m_F_surface.rows(), [this](size_t k) { return m_F_surface.row(k); });
    msh.add_physical_group("EnvelopeSurface");

    msh.save(filename, true);
}

void EmbedSurface::write_emb_vtu(const std::string& filename) const
{
    paraviewo::VTUWriter writer;
    for (size_t i = 0; i < m_T_tags.cols(); ++i) {
        writer.add_cell_field(fmt::format("tag_{}", i), m_T_tags.col(i).cast<double>());
    }
    writer.write_mesh(filename, m_V_emb, m_T_emb);
}

std::pair<Vector3d, Vector3d> EmbedSurface::bbox_minmax() const
{
    const Vector3d bbox_max = m_V_emb.colwise().maxCoeff();
    const Vector3d bbox_min = m_V_emb.colwise().minCoeff();

    return std::make_pair(bbox_min, bbox_max);
}

std::vector<Eigen::Vector3d> EmbedSurface::V_surf_to_vector() const
{
    std::vector<Eigen::Vector3d> verts;

    verts.resize(m_V_surface.rows());
    for (size_t i = 0; i < m_V_surface.rows(); ++i) {
        verts[i] = m_V_surface.row(i);
    }

    return verts;
}

std::vector<std::array<size_t, 3>> EmbedSurface::F_surf_to_vector() const
{
    std::vector<std::array<size_t, 3>> tris;

    tris.resize(m_F_surface.rows());
    for (size_t i = 0; i < m_F_surface.rows(); ++i) {
        tris[i][0] = m_F_surface(i, 0);
        tris[i][1] = m_F_surface(i, 1);
        tris[i][2] = m_F_surface(i, 2);
    }

    return tris;
}

void EmbedSurface::V_surf_from_vector(const std::vector<Eigen::Vector3d>& verts)
{
    const V_MAP V_surface(verts[0].data(), verts.size(), 3);
    m_V_surface = V_surface;
    assert(m_V_surface.rows() == verts.size());
    assert(m_V_surface.cols() == 3);
}

void EmbedSurface::F_surf_from_vector(const std::vector<std::array<size_t, 3>>& tris)
{
    m_F_surface.resize(tris.size(), 3);
    for (size_t i = 0; i < tris.size(); ++i) {
        m_F_surface(i, 0) = tris[i][0];
        m_F_surface(i, 1) = tris[i][1];
        m_F_surface(i, 2) = tris[i][2];
    }
}

} // namespace wmtk::components::image_simulation