#include <igl/predicates/ear_clipping.h>
#include <fstream>
#include <set>
#include <wmtk/utils/Concurrency.hpp>
#include <wmtk/utils/predicates.hpp>
#include "TetWildMesh.h"

#include <atomic>
#include <mutex>

namespace wmtk::components::tetwild {

void TetWildMesh::output_embedded_polygon_mesh(
    std::string output_dir,
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::vector<size_t>>& polygon_faces,
    const std::vector<std::vector<size_t>>& polygon_cells,
    const std::vector<bool>& polygon_faces_on_input)
{
    assert(polygon_faces.size() == polygon_faces_on_input.size());

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

    for (size_t i = 0; i < polygon_faces_on_input.size(); i++) {
        output << "s " << polygon_faces_on_input[i] << std::endl;
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

void TetWildMesh::output_embedded_polygon_surface_mesh(
    std::string output_dir,
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::vector<size_t>>& polygon_faces,
    const std::vector<bool>& polygon_faces_on_input)
{
    assert(polygon_faces.size() == polygon_faces_on_input.size());

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
        if (!polygon_faces_on_input[i]) continue;
        output << "f ";
        for (int j = 0; j < polygon_faces[i].size(); j++) {
            output << polygon_faces[i][j] + 1 << " ";
        }
        output << std::endl;
    }

    output.close();
}


void TetWildMesh::output_tetrahedralized_embedded_mesh(
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

void TetWildMesh::output_init_tetmesh(std::string output_dir)
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

void TetWildMesh::output_tracked_surface(std::string output_file)
{
    logger().info("Write {}", output_file);
    output_faces(output_file, [](auto& f) { return f.m_is_surface_fs; });
}


bool TetWildMesh::check_polygon_face_validity(std::vector<Vector3r> points)
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

std::vector<std::array<size_t, 3>> TetWildMesh::triangulate_polygon_face(
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
                    {size_t(cur.second), size_t(next.second), size_t(nextnext.second)}};
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
            {size_t(points_vector[0].second),
             size_t(points_vector[1].second),
             size_t(points_vector[points_vector.size() - 1].second)}};
        triangulated_faces.push_back(t);
        points_vector.erase(points_vector.begin());
    }

    return triangulated_faces;
}

// ---------------------------------------------------------------------------
// insertion_by_volumeremesher_old
//
// Conformally insert an input triangle surface mesh into a tetrahedral
// background mesh using the external "VolumeRemesher" library
// (vol_rem::embed_tri_in_poly_mesh, Cherchi et al.). The output is a
// tetrahedralization of the (bounding-box padded) domain whose faces exactly
// conform to the input surface, expressed in exact rational coordinates,
// together with the bookkeeping needed downstream: which faces lie on the input
// surface and which vertices lie on the input surface.
//
// High-level pipeline:
//   1. Build a Delaunay background tet mesh of the bbox-padded input points.
//   2. Flatten the background tets and the input triangles into the plain
//      double / uint32 arrays the remesher expects.
//   3. Call embed_tri_in_poly_mesh, which computes the exact arrangement of the
//      input triangles cut into the background mesh and returns it as a
//      *polygonal* complex: rational vertices, polygon faces, and polyhedral
//      cells, plus the list of which faces sit on the input surface.
//   4. Decode that polygonal complex, triangulate every polygon face, and
//      tetrahedralize every polyhedral cell (trivially when it is already a
//      tet, otherwise by coning all of its triangulated faces to its centroid).
//   5. Emit the triangle faces, the tets, and the on-surface tags for faces and
//      vertices.
//
// This "_old" variant tetrahedralizes the polygonal cells itself (steps 4-5) by
// coning each cell to a per-cell centroid. It is kept for reference; the default
// path is now insertion_by_volumeremesher, which consumes the tetrahedralization
// the remesher returns directly. See the header comment on
// insertion_by_volumeremesher below for a side-by-side of the differences.
//
// Parameters:
//   vertices                  [in]  input surface vertices (double precision)
//   faces                     [in]  input surface triangles (indices into vertices)
//   v_rational                [out] all output vertices, exact rational coords
//   facets_after              [out] triangulated faces of the arrangement
//   is_v_on_input             [out] per-vertex: is this vertex on the input surface
//   tets_after                [out] output tetrahedra (indices into v_rational)
//   tet_face_on_input_surface [out] flat per-tet-face flags, 4 per tet in WMTK
//                                   local face order, marking on-surface faces
// ---------------------------------------------------------------------------

// we have the vertices and triangles
// to generate what we need for volumemesher
// coords, ncoords, tri_idx, ntri_idx

// embed input surface on generated back ground mesh

void TetWildMesh::insertion_by_volumeremesher_old(
    const std::vector<Vector3d>& vertices, // input surface vertices (double)
    const std::vector<std::array<size_t, 3>>& faces, // input surface triangles
    std::vector<Vector3r>& v_rational, // out: arrangement vertices (rational)
    std::vector<std::array<size_t, 3>>& facets_after, // out: triangulated faces
    std::vector<bool>& is_v_on_input, // out: vertex-on-input-surface flags
    std::vector<std::array<size_t, 4>>& tets_after, // out: output tets
    std::vector<bool>& tet_face_on_input_surface) // out: 4 face-on-surface flags per tet
{
    logger().info("Insertion #v = {}, #f = {}", vertices.size(), faces.size());

    // Step 1: build the background mesh. init_from_delaunay_box_mesh pads the
    // input points with a bounding box and computes a Delaunay
    // tetrahedralization, storing both float (m_posf) and rational (m_pos)
    // vertex positions.
    // generate background mesh
    init_from_delaunay_box_mesh(vertices);
    // output_mesh("background_tetmesh.msh");
    // std::cout << "background_tetmesh written!" << std::endl;

    // Step 2: flatten the background tet mesh into the plain arrays the remesher
    // consumes. tet_ver_coord is xyz-interleaved vertex coordinates (double) and
    // tet_index is 4 vertex ids per tet.
    // prepare tet vertices and tet index info

    auto tet_vers = get_vertices();
    auto tets = get_tets();
    std::vector<double> tet_ver_coord(3 * tet_vers.size());
    std::vector<uint32_t> tet_index(4 * tets.size());
    logger().info("after delaunay #V = {}, #T = {}", tet_vers.size(), tets.size());
    // tet_ver_coord.reserve(3 * tet_vers.size());
    // tet_index.reserve(4 * tets.size());

    // vertex coordinates: use the double positions (m_posf); vertex i occupies
    // slots [3*i, 3*i+2]. The mesh was just built, so vertex ids are contiguous
    // and m_vertex_attribute can be indexed directly by i.
    for (int i = 0; i < tet_vers.size(); ++i) {
        tet_ver_coord[3 * i] = m_vertex_attribute[i].m_posf[0];
        tet_ver_coord[3 * i + 1] = m_vertex_attribute[i].m_posf[1];
        tet_ver_coord[3 * i + 2] = m_vertex_attribute[i].m_posf[2];
    }

    // tet connectivity: oriented_tet_vids returns the 4 vertex ids in positive
    // orientation, stored as 4 consecutive entries per tet.
    for (int i = 0; i < tets.size(); ++i) {
        auto tet_vids = oriented_tet_vids(tets[i]);
        tet_index[4 * i] = (int)tet_vids[0];
        tet_index[4 * i + 1] = (int)tet_vids[1];
        tet_index[4 * i + 2] = (int)tet_vids[2];
        tet_index[4 * i + 3] = (int)tet_vids[3];
    }

    // Same flattening for the input surface: tri_ver_coord is xyz-interleaved
    // vertex coordinates (double) and tri_index is 3 vertex ids per triangle.
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
        tri_index[3 * i] = (int)faces[i][0];
        tri_index[3 * i + 1] = (int)faces[i][1];
        tri_index[3 * i + 2] = (int)faces[i][2];
    }

    // {
    //     logger().warn("Write input.off");
    //     std::ofstream off("input.off");
    //     off << "OFF\n";
    //     off << vertices.size() << " " << faces.size() << " 0\n";
    //     for(const auto& v : vertices){
    //         off << v[0] << " " << v[1] << " " << v[2] << std::endl;
    //     }
    //     for(const auto& f : faces){
    //         logger().info("{}", f);
    //         off << f.size() << " ";
    //         for(size_t vid : f){
    //             logger().info("{}", vid);
    //             off << vid << " ";
    //         }
    //         off << std::endl;
    //     }
    //     off.close();
    // }

    // std::cout << tri_ver_coord.size() << std::endl;
    // std::cout << tri_index.size() << std::endl;
    // std::cout << tet_ver_coord.size() << std::endl;
    // std::cout << tet_index.size() << std::endl;

    // Outputs of the remesher (step 3). The arrangement is returned as a
    // polygonal complex:
    //   embedded_vertices        exact rational coords, xyz-interleaved
    //   embedded_facets          polygon faces, size-prefixed: [n, v0..v(n-1), ...]
    //   embedded_cells           polyhedral cells, size-prefixed by face count:
    //                            [m, f0..f(m-1), ...] (indices into the faces)
    //   embedded_facets_on_input indices (into the decoded polygon face list) of
    //                            the faces that lie on the input surface
    std::vector<vol_rem::bigrational> embedded_vertices;
    std::vector<uint32_t> embedded_facets;
    std::vector<uint32_t> embedded_cells;
    std::vector<uint32_t> embedded_facets_on_input;

    // These describe an alternative tet-based output of the remesher. They are
    // filled in by embed_tri_in_poly_mesh but only consumed by the newer
    // insertion_by_volumeremesher; this "_old" routine tetrahedralizes the
    // polygonal cells (embedded_cells) itself and ignores them.
    std::vector<std::array<uint32_t, 4>> out_tets;
    std::vector<uint32_t> final_tets_parent;
    std::vector<bool> cells_with_faces_on_input;
    std::vector<std::vector<uint32_t>> final_tets_parent_faces;

    // Sanity check the flattened inputs before handing them to the remesher:
    // warn about collinear (degenerate) input triangles and coplanar
    // (degenerate) background tets. These are warnings only; embedding proceeds.
    if (m_params.perform_sanity_checks) {
        logger().info("Check degenerate before embedding...");
        for (int i = 0; i < tri_index.size(); i += 3) {
            int id0 = tri_index[i + 0];
            int id1 = tri_index[i + 1];
            int id2 = tri_index[i + 2];
            Eigen::Vector3d v0;
            Eigen::Vector3d v1;
            Eigen::Vector3d v2;
            v0 << tri_ver_coord[3 * id0 + 0], tri_ver_coord[3 * id0 + 1],
                tri_ver_coord[3 * id0 + 2];
            v1 << tri_ver_coord[3 * id1 + 0], tri_ver_coord[3 * id1 + 1],
                tri_ver_coord[3 * id1 + 2];
            v2 << tri_ver_coord[3 * id2 + 0], tri_ver_coord[3 * id2 + 1],
                tri_ver_coord[3 * id2 + 2];

            if (utils::predicates::is_degenerate(v0, v1, v2)) {
                logger().warn(
                    "Face ({}, {}, {}) is collinear!",
                    v0.transpose(),
                    v1.transpose(),
                    v2.transpose());
            }
        }

        for (int i = 0; i < tet_index.size(); i += 4) {
            int id0 = tet_index[i + 0];
            int id1 = tet_index[i + 1];
            int id2 = tet_index[i + 2];
            int id3 = tet_index[i + 3];
            Eigen::Vector3d v0;
            Eigen::Vector3d v1;
            Eigen::Vector3d v2;
            Eigen::Vector3d v3;
            v0 << tet_ver_coord[3 * id0 + 0], tet_ver_coord[3 * id0 + 1],
                tet_ver_coord[3 * id0 + 2];
            v1 << tet_ver_coord[3 * id1 + 0], tet_ver_coord[3 * id1 + 1],
                tet_ver_coord[3 * id1 + 2];
            v2 << tet_ver_coord[3 * id2 + 0], tet_ver_coord[3 * id2 + 1],
                tet_ver_coord[3 * id2 + 2];
            v3 << tet_ver_coord[3 * id3 + 0], tet_ver_coord[3 * id3 + 1],
                tet_ver_coord[3 * id3 + 2];

            if (utils::predicates::is_degenerate(v0, v1, v2, v3)) {
                logger().warn(
                    "Tet ({}, {}, {}) is coplanar!",
                    v0.transpose(),
                    v1.transpose(),
                    v2.transpose(),
                    v3.transpose());
            }
        }
        logger().info("done");
    }

    // Step 3: compute the exact arrangement. This intersects the input triangles
    // with the background tets and returns the conforming polygonal complex in
    // the embedded_* / out_* outputs declared above. The trailing `true`
    // requests the on-input-surface tagging (embedded_facets_on_input).
    // volumeremesher embed
    // PR #12 (wildmeshing/VolumeRemesher) added input edges/points and per-simplex
    // provenance outputs. tetwild only embeds triangles, so the edge/point inputs
    // are empty and the provenance outputs are unused.
    std::vector<double> vr_edge_coords, vr_point_coords;
    std::vector<uint32_t> vr_edge_indexes;
    std::vector<std::vector<std::array<uint32_t, 4>>> vr_tri_provenance;
    std::vector<std::vector<std::array<uint32_t, 3>>> vr_edge_provenance;
    std::vector<std::array<uint32_t, 2>> vr_point_provenance;
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
        vr_edge_coords,
        vr_edge_indexes,
        vr_point_coords,
        vr_tri_provenance,
        vr_edge_provenance,
        vr_point_provenance,
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

    // Step 4a: copy the arrangement vertices into wmtk rational Vector3r. The
    // remesher stores them as vol_rem::bigrational; convert exactly (no rounding).
    // The backend differs by build: with the GNU GMP C++ classes we hand over the
    // mpq_t directly, otherwise we round-trip through the binary string form.
    for (int i = 0; i < embedded_vertices.size() / 3; i++) {
        v_rational.push_back(Vector3r());
#ifdef USE_GNU_GMP_CLASSES
        v_rational.back()[0].init(embedded_vertices[3 * i].get_mpq_t());
        v_rational.back()[1].init(embedded_vertices[3 * i + 1].get_mpq_t());
        v_rational.back()[2].init(embedded_vertices[3 * i + 2].get_mpq_t());
#else
        v_rational.back()[0].init_from_bin(embedded_vertices[3 * i].get_str());
        v_rational.back()[1].init_from_bin(embedded_vertices[3 * i + 1].get_str());
        v_rational.back()[2].init_from_bin(embedded_vertices[3 * i + 2].get_str());
#endif
    }

    // Step 4b: decode the size-prefixed embedded_facets array into an explicit
    // list of polygon faces. Layout: at position i, embedded_facets[i] is the
    // vertex count n of the face, followed by its n vertex ids; then jump past
    // them (i += polysize, plus the loop's i++) to the next face's count.
    // Faces may have more than 3 vertices (the arrangement produces polygons).
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

    // Debug check: any polygon face that is already a triangle must be
    // non-degenerate. Report (but do not fix) collinear triangles.
    // test polygon faces - only the triangles
    for (int i = 0; i < polygon_faces.size(); ++i) {
        if (polygon_faces[i].size() != 3) {
            continue;
        }

        const Vector3r v0 = v_rational[polygon_faces[i][0]];
        const Vector3r v1 = v_rational[polygon_faces[i][1]];
        const Vector3r v2 = v_rational[polygon_faces[i][2]];
        const Vector3r v01 = (v1 - v0);
        const Vector3r v02 = (v2 - v0);
        const Vector3r r = v01.cross(v02);
        if (r[0] == 0 && r[1] == 0 && r[2] == 0) {
            logger().error("Collinear triangle in polygon faces. ID = {}", i);
            logger().error("v0 = {}", to_double(v0));
            logger().error("v1 = {}", to_double(v1));
            logger().error("v2 = {}", to_double(v2));
        }
    }

    // Step 4c: decode the size-prefixed embedded_cells array the same way, but
    // each cell is a list of *face* indices (into polygon_faces) rather than
    // vertex ids. tets_final will collect the tetrahedra produced from the cells.
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

    logger().info("#polygon cells = {}", polygon_cells.size());

    // check polygon face validity
    // only for debug use
    // for (int i = 0; i < polygon_faces.size(); i++) {
    //     std::vector<Vector3r> polyface_coordinates;
    //     for (int j = 0; j < polygon_faces[i].size(); j++) {
    //         polyface_coordinates.push_back(v_rational[polygon_faces[i][j]]);
    //     }
    //     if (!check_polygon_face_validity(polyface_coordinates)) {
    //         std::cout << "polygon face invalid!!: " << i << std::endl;
    //     }
    // }

    // Build a per-polygon-face flag marking faces that lie on the input surface.
    // embedded_facets_on_input holds the indices of those faces.
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
    // output_embedded_polygon_surface_mesh(
    //     "surface_polygon_mesh_after_insertion.obj",
    //     v_rational,
    //     polygon_faces,
    //     polygon_faces_on_input_surface);

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

    // Step 4d: triangulate every polygon face. triangulated_faces accumulates
    // the resulting triangles; triangulated_faces_on_input mirrors the
    // on-surface flag for each; map_poly_to_tri_face[i] records which triangle
    // indices came from polygon face i (needed later to rebuild each cell's
    // triangular faces during tetrahedralization).
    std::vector<std::array<size_t, 3>> triangulated_faces;
    std::vector<bool> triangulated_faces_on_input;
    std::vector<std::vector<size_t>> map_poly_to_tri_face(polygon_faces.size());

    int poly_cnt = 0;

    // triangulate polygon faces
    for (int i = 0; i < polygon_faces.size(); i++) {
        // Skip faces already triangulated. Each face is visited once here, so
        // this is a safety net against processing the same face twice.
        // already clipped in other polygon
        if (map_poly_to_tri_face[i].size() != 0) continue;

        // new polygon face to clip
        std::vector<std::array<size_t, 3>> clipped_indices;
        std::vector<Vector3r> poly_coordinates;
        std::vector<size_t> polygon_face = polygon_faces[i];
        assert(polygon_face.size() >= 3);

        if (polygon_face.size() == 3) {
            // Already a triangle: copy it straight through and inherit its
            // on-surface flag.
            // already a triangle
            std::array<size_t, 3> triangle_face = {
                {polygon_face[0], polygon_face[1], polygon_face[2]}};
            size_t idx = triangulated_faces.size();
            triangulated_faces.push_back(triangle_face);
            if (polygon_faces_on_input_surface[i]) {
                triangulated_faces_on_input.push_back(true);
            } else {
                triangulated_faces_on_input.push_back(false);
            }
            map_poly_to_tri_face[i].push_back(idx);
        } else {
            // Convex polygon (>3 vertices): gather its rational coordinates and
            // triangulate. triangulate_polygon_face returns triangles as *local*
            // indices into polygon_face, which are remapped back to global ids.
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
                    {polygon_face[clipped_indices[j][0]],
                     polygon_face[clipped_indices[j][1]],
                     polygon_face[clipped_indices[j][2]]}};
                // std::cout<<triangle_face[0]<<" "<<triangle_face[1]<<"
                // "<<triangle_face[2]<<std::endl;
                size_t idx = triangulated_faces.size();
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

    logger().info("#triangulated polygons = {}", poly_cnt);

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

    // Step 4e: tetrahedralize every polyhedral cell into tets_final, tagging
    // each resulting tet's 4 faces with whether they lie on the input surface.
    // tetrahedralize cells

    // track face on surface per tet
    // std::vector<bool> tet_face_on_input_surface;

    int was_tet_cnt = 0; // number of cells that were already tetrahedra
    for (int i = 0; i < polygon_cells.size(); i++) {
        auto polygon_cell = polygon_cells[i];

        // gather the cell's unique vertices from all of its faces
        // get polygon vertices
        std::vector<size_t> polygon_vertices;
        for (auto f : polygon_cell) {
            for (auto v : polygon_faces[f]) {
                polygon_vertices.push_back(v);
            }
        }
        wmtk::vector_unique(polygon_vertices);

        // total number of triangles across all of the cell's (triangulated)
        // faces. A cell bounded by exactly 4 triangles is already a tetrahedron.
        // compute number of triangle faces
        size_t num_faces = 0;
        for (size_t f : polygon_cell) {
            num_faces += map_poly_to_tri_face[f].size();
        }

        // Case A: the cell is already a tetrahedron (4 triangular faces). Emit it
        // directly, no Steiner point needed.
        // polygon already a tet
        if (num_faces == 4) {
            was_tet_cnt++;
            assert(polygon_vertices.size() == 4);
            // Take the first face as (v0, v1, v2) and pick the apex v3 as the one
            // vertex of the second face not shared with the first.
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

            std::array<size_t, 4> tetra = {{v0, v1, v2, v3}};

            // Enforce positive orientation: if the signed volume is negative,
            // swap v0 and v1. Uses exact rational arithmetic.
            // if inverted then fix the orientation
            Vector3r v0v1 = v_rational[v1] - v_rational[v0];
            Vector3r v0v2 = v_rational[v2] - v_rational[v0];
            Vector3r v0v3 = v_rational[v3] - v_rational[v0];
            if ((v0v1.cross(v0v2)).dot(v0v3) < 0) {
                tetra = {{v1, v0, v2, v3}};
            }

            // push the tet to final queue;
            tets_final.push_back(tetra);

            // The four faces in WMTK local order: f0=(v0,v1,v2), f1=(v0,v2,v3),
            // f2=(v0,v1,v3), f3=(v1,v2,v3). Compared as unordered vertex sets so
            // each of the cell's faces can be matched to its local slot.
            std::set<size_t> local_f1 = {tetra[0], tetra[1], tetra[2]};
            std::set<size_t> local_f2 = {tetra[0], tetra[2], tetra[3]};
            std::set<size_t> local_f3 = {tetra[0], tetra[1], tetra[3]};
            std::set<size_t> local_f4 = {tetra[1], tetra[2], tetra[3]};

            // For each of the cell's faces, find which local slot it is and copy
            // that face's on-input flag into the matching position.
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

        // Case B: general polyhedral cell. Add a Steiner point at the cell
        // centroid and cone (connect) every boundary triangle to it, producing
        // one tet per boundary triangle. The centroid is a new rational vertex
        // appended to v_rational. Note: this is why the "_old" path introduces
        // extra interior vertices that the remesher's own tets would not have.
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
                // Tet = boundary triangle + centroid apex.
                std::array<size_t, 4> tetra = {
                    {triangulated_faces[t][0],
                     triangulated_faces[t][1],
                     triangulated_faces[t][2],
                     centroid_idx}};
                // std::sort(tetra.begin(), tetra.end());
                // Enforce positive orientation (swap the first two base vertices
                // if the signed volume is negative).
                // check inverted tet and fix
                Vector3r v0v1 = v_rational[tetra[1]] - v_rational[tetra[0]];
                Vector3r v0v2 = v_rational[tetra[2]] - v_rational[tetra[0]];
                Vector3r v0v3 = v_rational[tetra[3]] - v_rational[tetra[0]];
                if ((v0v1.cross(v0v2)).dot(v0v3) < 0) {
                    tetra = {
                        {triangulated_faces[t][1],
                         triangulated_faces[t][0],
                         triangulated_faces[t][2],
                         centroid_idx}};
                }

                // Only the base face (local index 0, the original boundary
                // triangle) can lie on the input surface; the three side faces
                // all touch the interior centroid and never do, hence false.
                tets_final.push_back(tetra);
                tet_face_on_input_surface.push_back(triangulated_faces_on_input[t]);
                tet_face_on_input_surface.push_back(false);
                tet_face_on_input_surface.push_back(false);
                tet_face_on_input_surface.push_back(false);
            }
        }
    }

    // std::cout << "polygon was tet num: " << was_tet_cnt << std::endl;
    // std::cout << "vertices final num: " << v_rational.size() << std::endl;
    // std::cout << "tets final num: " << tets_final.size() << std::endl;
    // std::cout << "track face size: " << tet_face_on_input_surface.size() << std::endl;

    // Step 5: publish the results. Note v_rational now also contains the centroid
    // Steiner points appended above, so it is the full output vertex set.
    facets_after = triangulated_faces;
    tets_after = tets_final;

    // A vertex is on the input surface iff it is a corner of some triangulated
    // face that lies on the input surface. Initialize all to false, then flag the
    // three corners of every on-surface face.
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
    logger().info("{} of {} vertices are on the surface", on_surface_v_cnt, is_v_on_input.size());

    // Final sanity check: every output tet must be positively oriented, i.e. the
    // signed volume ((p1-p0)x(p2-p0)).(p3-p0) must be strictly positive. This
    // exact rational test catches any tet the per-tet orientation fixes missed.
    for (const auto& t : tets_after) {
        const Vector3r p0 = v_rational[t[0]];
        const Vector3r p1 = v_rational[t[1]];
        const Vector3r p2 = v_rational[t[2]];
        const Vector3r p3 = v_rational[t[3]];
        Vector3r n = (p1 - p0).cross(p2 - p0);
        Vector3r d = p3 - p0;
        auto res = n.dot(d);
        if (res <= 0) {
            logger().error("Inverted tet {}", t);
        }
    }


    // {
    //     // write triangulated faces to file
    //     MatrixXd V;
    //     V.resize(v_rational.size(), 3);
    //     for(size_t i = 0; i < v_rational.size(); ++i){
    //         V.row(i) = to_double(v_rational[i]);
    //     }

    //     size_t f_count = 0;
    //     for(size_t i = 0; i < triangulated_faces_on_input.size(); ++i){
    //         if(triangulated_faces_on_input[i]){
    //             f_count++;
    //         }
    //     }

    //     MatrixXi F;
    //     F.resize(f_count, 3);
    //     f_count = 0;
    //     for(size_t i = 0; i < facets_after.size(); ++i){
    //         if(!triangulated_faces_on_input[i]){
    //             continue;
    //         }
    //         F(f_count,0) = facets_after[i][0];
    //         F(f_count,1) = facets_after[i][1];
    //         F(f_count,2) = facets_after[i][2];
    //         f_count++;
    //     }

    //     logger().warn("Write traced.off");
    //     igl::writeOFF("traced.off", V, F);
    // }
    // {
    //     // polygon_faces_on_input_surface
    //     // polygon_faces

    //     MatrixXd V;
    //     V.resize(v_rational.size(), 3);
    //     for(size_t i = 0; i < v_rational.size(); ++i){
    //         V.row(i) = to_double(v_rational[i]);
    //     }

    //     std::vector<std::vector<size_t>> faces;
    //     for(size_t i = 0; i < polygon_faces.size(); ++i){
    //         if(!polygon_faces_on_input_surface[i]){
    //             continue;
    //         }
    //         faces.push_back(polygon_faces[i]);
    //     }

    //     logger().warn("Write polygons.off");
    //     std::ofstream off("polygons.off");
    //     off << "OFF\n";
    //     off << V.rows() << " " << faces.size() << " 0\n";
    //     for(size_t i = 0; i < V.rows(); ++i){
    //         off << V(i,0) << " " << V(i,1) << " " << V(i,2) << std::endl;
    //     }
    //     for(const auto& f : faces){
    //         off << f.size() << " ";
    //         for(size_t vid : f){
    //             off << vid << " ";
    //         }
    //         off << std::endl;
    //     }
    //     off.close();
    // }
}


// ---------------------------------------------------------------------------
// insertion_by_volumeremesher  (the fast variant -- the default path)
//
// Same goal as insertion_by_volumeremesher_old: conformally insert the input
// surface into a background tet mesh via vol_rem::embed_tri_in_poly_mesh. Steps
// 1-3 (build background mesh, flatten inputs, run the remesher) are identical.
// The difference is entirely in what it does with the remesher output.
//
// This is the variant actually used. It consumes the tetrahedralization the
// remesher now returns directly (see the VolumeRemesher changes that emit a full
// conforming tetrahedralization with surface-tracking metadata), which is
// cheaper and adds no interior Steiner points. The older "_old" routine, which
// re-tetrahedralizes the polygonal cells by coning to per-cell centroids, is
// kept for reference.
//
// How it differs from insertion_by_volumeremesher_old (steps 4-5):
//   * Tets come straight from the remesher. It consumes the remesher's own
//     tetrahedra (out_tets) directly instead of re-tetrahedralizing the
//     polygonal cells. Consequently it adds NO centroid Steiner points -- the
//     output has fewer, better-shaped tets and no extra interior vertices.
//   * Facets are already triangles. The remesher triangulates every face, so
//     embedded_facets is parsed with a fixed stride of 4 (1 size prefix + 3
//     vertex ids), whereas "_old" triangulates arbitrary polygon faces itself.
//   * Surface tracking uses remesher metadata. It relies on final_tets_parent
//     (which polygonal cell each output tet came from), final_tets_parent_faces
//     (which polygon faces bound that tet) and cells_with_faces_on_input (a fast
//     skip for cells that touch no input face), instead of "_old"'s per-cell
//     unordered-set face matching.
//   * Vertices are compacted. Vertices left unreferenced by out_tets are removed
//     and all indices are remapped (see the "removing unreferenced vertices"
//     block); "_old" keeps every vertex.
//   * Orientation is passed through. The remesher already emits WMTK-positively
//     oriented tets, so out_tets is copied straight into tets_after (only the 4
//     per-tet face flags are reordered to WMTK's local face order).
//
// The `polygon_faces` OUTPUT parameter here is a list of triangles
// (std::array<size_t,3>), reflecting that facets are already triangles; in
// "_old" the corresponding internal structure was a variable-length polygon list.
// ---------------------------------------------------------------------------
void TetWildMesh::insertion_by_volumeremesher(
    const std::vector<Vector3d>& vertices, // input surface vertices (double)
    const std::vector<std::array<size_t, 3>>& faces, // input surface triangles
    std::vector<Vector3r>& v_rational, // out: arrangement vertices (rational)
    std::vector<std::array<size_t, 3>>& polygon_faces, // out: triangular facets
    std::vector<bool>& is_v_on_input, // out: vertex-on-input-surface flags
    std::vector<std::array<size_t, 4>>& tets_after, // out: output tets
    std::vector<bool>& tet_face_on_input_surface) // out: 4 face-on-surface flags per tet
{
    std::cout << "vertices size: " << vertices.size() << std::endl;
    std::cout << "faces size: " << faces.size() << std::endl;

    // Step 1: build the Delaunay background mesh (same as the "_old" variant).
    // generate background mesh
    init_from_delaunay_box_mesh(vertices);
    // output_mesh("background_tetmesh.msh");
    // std::cout << "background_tetmesh written!" << std::endl;

    // Sanity check: the freshly built Delaunay tets should all be well oriented.
    for (const Tuple& t : get_tets()) {
        if (is_inverted(t)) {
            logger().error("Tet {} is inverted after init from delaunay!", t.tid(*this));
        }
    }

    // Step 2: flatten the background tet mesh into plain double / uint32 arrays
    // (xyz-interleaved coords, 4 vertex ids per tet), exactly as in "_old".
    // prepare tet vertices and tet index info

    auto tet_vers = get_vertices();
    auto tets = get_tets();
    std::vector<double> tet_vrt_coord(3 * tet_vers.size());
    std::vector<uint32_t> tet_indices(4 * tets.size());
    std::cout << "tetver size: " << tet_vers.size() << std::endl;
    std::cout << "tet size: " << tets.size() << std::endl;

    for (int i = 0; i < tet_vers.size(); ++i) {
        tet_vrt_coord[3 * i] = m_vertex_attribute[i].m_posf[0];
        tet_vrt_coord[3 * i + 1] = m_vertex_attribute[i].m_posf[1];
        tet_vrt_coord[3 * i + 2] = m_vertex_attribute[i].m_posf[2];
    }

    for (int i = 0; i < tets.size(); ++i) {
        auto tet_vids = oriented_tet_vids(tets[i]);
        tet_indices[4 * i] = (int)tet_vids[0];
        tet_indices[4 * i + 1] = (int)tet_vids[1];
        tet_indices[4 * i + 2] = (int)tet_vids[2];
        tet_indices[4 * i + 3] = (int)tet_vids[3];
    }

    // Step 2 (cont.): flatten the input surface the same way.
    // prepare input surfaces info
    std::vector<double> tri_vrt_coord(3 * vertices.size());
    std::vector<uint32_t> triangle_indices(3 * faces.size());

    for (int i = 0; i < vertices.size(); ++i) {
        tri_vrt_coord[3 * i] = vertices[i][0];
        tri_vrt_coord[3 * i + 1] = vertices[i][1];
        tri_vrt_coord[3 * i + 2] = vertices[i][2];

        // Leftover debugging probe: warn when an input vertex matches a specific
        // hard-coded location (projected onto the yz-plane). Harmless, and would
        // be removed in a cleanup.
        Eigen::Vector3d t1(6.808340025785895, 22.618768054579625, 0);
        t1[0] = 0;
        Eigen::Vector3d t2 = vertices[i];
        t2[0] = 0;
        if ((t2 - t1).norm() < 1e-4) {
            logger().warn("Found {}", vertices[i].transpose());
        }
    }

    for (int i = 0; i < faces.size(); ++i) {
        triangle_indices[3 * i] = (int)faces[i][0];
        triangle_indices[3 * i + 1] = (int)faces[i][1];
        triangle_indices[3 * i + 2] = (int)faces[i][2];
    }

    std::cout << tri_vrt_coord.size() << std::endl;
    std::cout << triangle_indices.size() << std::endl;
    std::cout << tet_vrt_coord.size() << std::endl;
    std::cout << tet_indices.size() << std::endl;

    // Remesher outputs. Unlike "_old", this variant actually uses the tet-based
    // outputs: out_tets (the remesher's tetrahedra), final_tets_parent (parent
    // polyhedral cell of each tet), cells_with_faces_on_input (per-cell flag),
    // and final_tets_parent_faces (the parent faces bounding each tet). The
    // embedded_cells polygonal-cell decoding used by "_old" is not needed here.
    std::vector<vol_rem::bigrational> embedded_vertices;
    std::vector<uint32_t> embedded_facets;
    std::vector<uint32_t> embedded_cells;
    std::vector<uint32_t> embedded_facets_on_input;

    std::vector<std::array<uint32_t, 4>> out_tets;
    std::vector<uint32_t> final_tets_parent;
    std::vector<bool> cells_with_faces_on_input;
    std::vector<std::vector<uint32_t>> final_tets_parent_faces;

    // Warn about degenerate (collinear) input triangles before embedding.
    {
        logger().warn("Check collinearity before embedding");
        for (int i = 0; i < triangle_indices.size(); i += 3) {
            int id0 = triangle_indices[i + 0];
            int id1 = triangle_indices[i + 1];
            int id2 = triangle_indices[i + 2];
            Eigen::Vector3d v0;
            Eigen::Vector3d v1;
            Eigen::Vector3d v2;
            v0 << tri_vrt_coord[3 * id0 + 0], tri_vrt_coord[3 * id0 + 1],
                tri_vrt_coord[3 * id0 + 2];
            v1 << tri_vrt_coord[3 * id1 + 0], tri_vrt_coord[3 * id1 + 1],
                tri_vrt_coord[3 * id1 + 2];
            v2 << tri_vrt_coord[3 * id2 + 0], tri_vrt_coord[3 * id2 + 1],
                tri_vrt_coord[3 * id2 + 2];

            if (utils::predicates::is_degenerate(v0, v1, v2)) {
                logger().error(
                    "Face ({}, {}, {}) is collinear!",
                    v0.transpose(),
                    v1.transpose(),
                    v2.transpose());
            }
        }
        logger().warn("Check done");
    }

    // Step 3: run the exact arrangement (identical call to the "_old" variant).
    // volumeremesher embed
    std::vector<double> vr_edge_coords, vr_point_coords;
    std::vector<uint32_t> vr_edge_indexes;
    std::vector<std::vector<std::array<uint32_t, 4>>> vr_tri_provenance;
    std::vector<std::vector<std::array<uint32_t, 3>>> vr_edge_provenance;
    std::vector<std::array<uint32_t, 2>> vr_point_provenance;
    vol_rem::embed_tri_in_poly_mesh(
        tri_vrt_coord,
        triangle_indices,
        tet_vrt_coord,
        tet_indices,
        embedded_vertices,
        embedded_facets,
        embedded_cells,
        out_tets,
        final_tets_parent,
        embedded_facets_on_input,
        cells_with_faces_on_input,
        final_tets_parent_faces,
        vr_edge_coords,
        vr_edge_indexes,
        vr_point_coords,
        vr_tri_provenance,
        vr_edge_provenance,
        vr_point_provenance,
        true);

    // Step 4a: copy the arrangement vertices to exact rational Vector3r (same as
    // "_old"). No compaction yet -- unused vertices are pruned near the end.
    for (int i = 0; i < embedded_vertices.size() / 3; i++) {
        v_rational.push_back(Vector3r());
#ifdef USE_GNU_GMP_CLASSES
        v_rational.back()[0].init(embedded_vertices[3 * i + 0].get_mpq_t());
        v_rational.back()[1].init(embedded_vertices[3 * i + 1].get_mpq_t());
        v_rational.back()[2].init(embedded_vertices[3 * i + 2].get_mpq_t());
#else
        v_rational.back()[0].init_from_bin(embedded_vertices[3 * i + 0].get_str());
        v_rational.back()[1].init_from_bin(embedded_vertices[3 * i + 1].get_str());
        v_rational.back()[2].init_from_bin(embedded_vertices[3 * i + 2].get_str());
#endif
    }

    // Debug-only sanity check: the remesher now returns tets already in the WMTK
    // orientation ((v1-v0)x(v2-v0).(v3-v0) > 0), so out_tets is used directly (no
    // orientation fix-up when filling tets_after below). This verification is
    // exact-rational and O(#tets) -- prohibitively expensive on large meshes --
    // so it is compiled out of release builds.
    if (m_params.perform_sanity_checks) {
        logger().info("Check tet orientation after embedding...");
        for (const auto& vids : out_tets) {
            Vector3r n = (v_rational[vids[1]] - v_rational[vids[0]])
                             .cross(v_rational[vids[2]] - v_rational[vids[0]]);
            Vector3r d = v_rational[vids[3]] - v_rational[vids[0]];
            auto res = n.dot(d);
            if (res <= 0) {
                logger().error(
                    "After embed_tri_in_poly_mesh: Tet {} is inverted! res = {}",
                    vids,
                    res.to_double());
                if (res == 0) {
                    logger().error("Tet has 0 volume.");
                }
                logger().error("v0 = {}", to_double(v_rational[vids[0]]).transpose());
                logger().error("v1 = {}", to_double(v_rational[vids[1]]).transpose());
                logger().error("v2 = {}", to_double(v_rational[vids[2]]).transpose());
                logger().error("v3 = {}", to_double(v_rational[vids[3]]).transpose());
            }
        }
        logger().info("done");
    }


    // Step 4b: decode embedded_facets into triangles. KEY DIFFERENCE vs "_old":
    // here every facet must already be a triangle, so the array has a fixed
    // stride of 4 (1 size prefix + 3 vertex ids). If the remesher ever returns a
    // non-triangular facet this throws, rather than triangulating it like "_old".
    wmtk::logger().info("Facets loop...");
    //std::vector<std::array<size_t, 3>> polygon_faces; // index of vertices
    polygon_faces.reserve(embedded_facets.size() / 4);
    for (size_t i = 0; i < embedded_facets.size(); i += 4) {
        const size_t polysize = embedded_facets[i];
        if (polysize != 3) {
            log_and_throw_error("Facets must be triangles!");
        }
        std::array<size_t, 3> polygon;
        for (size_t j = 0; j < 3; ++j) {
            polygon[j] = embedded_facets[j + i + 1];
        }
        polygon_faces.push_back(polygon);
    }
    wmtk::logger().info("done");

    // Per-face on-input-surface flags, same as "_old".
    wmtk::logger().info("Tags loop...");
    std::vector<bool> polygon_faces_on_input(polygon_faces.size(), false);
    for (size_t i = 0; i < embedded_facets_on_input.size(); ++i) {
        polygon_faces_on_input[embedded_facets_on_input[i]] = true;
    }
    wmtk::logger().info("done");

    // Step 4c: surface tracking. KEY DIFFERENCE vs "_old": rather than matching
    // each cell's faces by unordered vertex sets after self-tetrahedralizing, it
    // uses the remesher-provided metadata. For each output tet it looks at its
    // parent cell (final_tets_parent) and the parent polygon faces bounding it
    // (final_tets_parent_faces), and marks the corresponding local tet face.
    wmtk::logger().info("tracking surface...");
    assert(final_tets_parent_faces.size() == out_tets.size());
    for (size_t i = 0; i < out_tets.size(); ++i) {
        const auto& tetra = out_tets[i];
        const uint32_t tetra_parent = final_tets_parent[i];

        // Fast path: if the parent cell has no faces on the input surface, none
        // of this tet's faces can either -- push four false flags and move on.
        if (!cells_with_faces_on_input[tetra_parent]) {
            for (int i = 0; i < 4; ++i) {
                tet_face_on_input_surface.push_back(false);
            }
            continue;
        }

        // The tet's four faces as sorted vertex sets, opposite each local vertex:
        // f0 opposite v0, f1 opposite v1, f2 opposite v2, f3 opposite v3. Sorting
        // lets us compare against each (also sorted) parent face by equality.
        // vector of std array and sort
        std::array<size_t, 3> local_f0{{tetra[1], tetra[2], tetra[3]}};
        std::sort(local_f0.begin(), local_f0.end());
        std::array<size_t, 3> local_f1{{tetra[0], tetra[2], tetra[3]}};
        std::sort(local_f1.begin(), local_f1.end());
        std::array<size_t, 3> local_f2{{tetra[0], tetra[1], tetra[3]}};
        std::sort(local_f2.begin(), local_f2.end());
        std::array<size_t, 3> local_f3{{tetra[0], tetra[1], tetra[2]}};
        std::sort(local_f3.begin(), local_f3.end());

        // For each parent face bounding this tet, find which local face it is and
        // copy that face's on-input flag into the matching slot.
        // track surface
        std::array<bool, 4> tet_face_on_input{{false, false, false, false}};
        for (const auto& f : final_tets_parent_faces[i]) {
            assert(polygon_faces[f].size() == 3);

            std::array<size_t, 3> f_vs = polygon_faces[f];
            std::sort(f_vs.begin(), f_vs.end());

            int64_t local_f_idx = -1;

            // decide which face it is

            if (f_vs == local_f0) {
                local_f_idx = 0;
            } else if (f_vs == local_f1) {
                local_f_idx = 1;
            } else if (f_vs == local_f2) {
                local_f_idx = 2;
            } else if (f_vs == local_f3) {
                local_f_idx = 3;
            }
            if (local_f_idx == -1) {
                log_and_throw_error("Could not find local index for tracked surface.");
            }

            tet_face_on_input[local_f_idx] = polygon_faces_on_input[f];
        }

        for (int k = 0; k < 4; k++) {
            tet_face_on_input_surface.push_back(tet_face_on_input[k]);
        }
    }

    // A vertex is on the input surface iff it is a corner of an on-input facet
    // (same rule as "_old").
    // track vertices on input
    is_v_on_input.resize(v_rational.size(), false);
    for (int i = 0; i < polygon_faces.size(); i++) {
        if (polygon_faces_on_input[i]) {
            is_v_on_input[polygon_faces[i][0]] = true;
            is_v_on_input[polygon_faces[i][1]] = true;
            is_v_on_input[polygon_faces[i][2]] = true;
        }
    }
    wmtk::logger().info("done");

    // Step 4d: compact the vertex set. KEY DIFFERENCE vs "_old", which keeps all
    // vertices (and even adds centroids): here the remesher's arrangement may
    // contain vertices not referenced by any output tet, so drop the unused ones.
    // Build v_map (old id -> new id) over the used vertices, rebuild the coord
    // and on-input arrays in the new numbering, then remap tets and facets.
    wmtk::logger().info("removing unreferenced vertices...");
    std::vector<bool> v_is_used_in_tet(v_rational.size(), false);
    for (const auto& t : out_tets) {
        for (const auto& v : t) {
            v_is_used_in_tet[v] = true;
        }
    }
    std::vector<int64_t> v_map(v_rational.size(), -1);
    std::vector<Vector3r> v_coords_final;
    std::vector<bool> is_v_on_input_buffer;

    for (size_t i = 0; i < v_rational.size(); ++i) {
        if (v_is_used_in_tet[i]) {
            v_map[i] = v_coords_final.size();
            v_coords_final.emplace_back(v_rational[i]);
            is_v_on_input_buffer.emplace_back(is_v_on_input[i]);
        }
    }
    // update vertices
    v_rational = v_coords_final;
    is_v_on_input = is_v_on_input_buffer;
    // update tets (in place, into the compacted numbering)
    for (auto& t : out_tets) {
        for (int i = 0; i < 4; ++i) {
            assert(v_map[t[i]] >= 0);
            t[i] = v_map[t[i]];
        }
    }
    // update polygon_faces (in place, into the compacted numbering)
    for (auto& t : polygon_faces) {
        for (int i = 0; i < 3; ++i) {
            assert(v_map[t[i]] >= 0);
            t[i] = v_map[t[i]];
        }
    }
    wmtk::logger().info("done");

    // Step 5: publish the tets. makeTetrahedra already emits WMTK-positively
    // oriented tets, so the vertices are copied straight through -- no swap. Only
    // the per-tet face flags need reordering: the tracking loop stored them in
    // "opposite-vertex" order (fl[k] = flag of the face opposite out_tets[i][k]),
    // which we map to WMTK's local face order.
    tets_after.resize(out_tets.size());
    for (size_t i = 0; i < out_tets.size(); ++i) {
        tets_after[i][0] = out_tets[i][0];
        tets_after[i][1] = out_tets[i][1];
        tets_after[i][2] = out_tets[i][2];
        tets_after[i][3] = out_tets[i][3];

        const bool fl0 = tet_face_on_input_surface[4 * i + 0]; // opp v0
        const bool fl1 = tet_face_on_input_surface[4 * i + 1]; // opp v1
        const bool fl2 = tet_face_on_input_surface[4 * i + 2]; // opp v2
        const bool fl3 = tet_face_on_input_surface[4 * i + 3]; // opp v3

        // WMTK local face order:
        //   local_f0: (v0, v1, v2) = opposite v3
        //   local_f1: (v0, v2, v3) = opposite v1
        //   local_f2: (v0, v1, v3) = opposite v2
        //   local_f3: (v1, v2, v3) = opposite v0
        tet_face_on_input_surface[4 * i + 0] = fl3;
        tet_face_on_input_surface[4 * i + 1] = fl1;
        tet_face_on_input_surface[4 * i + 2] = fl2;
        tet_face_on_input_surface[4 * i + 3] = fl0;
    }

    // Debug-only final sanity check: every published tet must be positively
    // oriented under the WMTK convention ((v1-v0)x(v2-v0)).(v3-v0) > 0. Exact-
    // rational and O(#tets), so it is compiled out of release builds.
#ifndef NDEBUG
    for (const auto& vids : tets_after) {
        Vector3r n = (v_rational[vids[1]] - v_rational[vids[0]])
                         .cross(v_rational[vids[2]] - v_rational[vids[0]]);
        Vector3r d = v_rational[vids[3]] - v_rational[vids[0]];
        auto res = n.dot(d);
        if (res <= 0) {
            logger().error("After insertion: Tet {} is inverted! res = {}", vids, res.to_double());
            // logger().error("v0 = {}", to_double(v_rational[vids[0]]).transpose());
            // logger().error("v1 = {}", to_double(v_rational[vids[1]]).transpose());
            // logger().error("v2 = {}", to_double(v_rational[vids[2]]).transpose());
            // logger().error("v3 = {}", to_double(v_rational[vids[3]]).transpose());
            // logger().error("res == 0 ? --> {}", res == 0);
        }
    }
#endif

    // Everything below is dead (commented-out) code: it is the "_old"-style
    // polygonal-cell tetrahedralization (decode embedded_cells, triangulate
    // faces, cone to centroids, track surface). It was left here for reference
    // while this variant was being switched over to the remesher's direct tets,
    // and is superseded by the live code above.
    // old code ...
    ////////////////////////////////////////////////////////////////////////////////////////

    // std::vector<std::vector<size_t>> polygon_cells;
    // std::vector<std::array<size_t, 4>> tets_final;
    // for (int i = 0; i < embedded_cells.size(); i++) {
    //     std::vector<size_t> polygon_cell;
    //     int cellsize = embedded_cells[i];
    //     for (int j = i + 1; j <= i + cellsize; j++) {
    //         polygon_cell.push_back(embedded_cells[j]);
    //     }
    //     polygon_cells.push_back(polygon_cell);
    //     i += cellsize;
    // }
    // std::cout << "polygon cells num: " << polygon_cells.size() << std::endl;


    // for (int i = 0; i < embedded_facets_on_input.size(); i++) {
    //     polygon_faces_on_input[embedded_facets_on_input[i]] = true;
    // }

    // std::vector<std::array<size_t, 3>> triangulated_faces;
    // std::vector<bool> triangulated_faces_on_input;
    // std::vector<std::vector<size_t>> map_poly_to_tri_face(polygon_faces.size());

    // int poly_cnt = 0;

    //// triangulate polygon faces
    // for (int i = 0; i < polygon_faces.size(); i++) {
    //    // already clipped in other polygon
    //    if (map_poly_to_tri_face[i].size() != 0) continue;

    //    std::array<size_t, 3> triangle_face = polygon_faces[i];
    //    int idx = triangulated_faces.size();
    //    triangulated_faces.push_back(triangle_face);
    //    if (polygon_faces_on_input[i]) {
    //        triangulated_faces_on_input.push_back(true);
    //    } else {
    //        triangulated_faces_on_input.push_back(false);
    //    }
    //    map_poly_to_tri_face[i].push_back(idx);
    //}

    // std::cout << "poly_cnt:" << poly_cnt << std::endl;

    // std::cout << "finish triangulation" << std::endl;

    // std::cout << "vertice before tetra num: " << v_rational.size() << std::endl;

    //// tetrahedralize cells

    //// track face on surface per tet
    //// std::vector<bool> tet_face_on_input_surface;

    // int was_tet_cnt = 0;
    // for (int i = 0; i < polygon_cells.size(); i++) {
    //     auto polygon_cell = polygon_cells[i];

    //    // get polygon vertices
    //    std::vector<size_t> polygon_vertices;
    //    for (auto f : polygon_cell) {
    //        for (auto v : polygon_faces[f]) {
    //            polygon_vertices.push_back(v);
    //        }
    //    }
    //    wmtk::vector_unique(polygon_vertices);

    //    // compute number of triangle faces
    //    int num_faces = 0;
    //    for (auto f : polygon_cell) {
    //        num_faces += map_poly_to_tri_face[f].size();
    //    }

    //    // polygon already a tet
    //    if (num_faces == 4) {
    //        was_tet_cnt++;
    //        assert(polygon_vertices.size() == 4);
    //        // get the correct orientation here
    //        size_t v0 = polygon_faces[polygon_cell[0]][0];
    //        size_t v1 = polygon_faces[polygon_cell[0]][1];
    //        size_t v2 = polygon_faces[polygon_cell[0]][2];
    //        size_t v3;
    //        for (auto v : polygon_faces[polygon_cell[1]]) {
    //            if (v != v0 && v != v1 && v != v2) {
    //                v3 = v;
    //                break;
    //            }
    //        }

    //        std::array<size_t, 4> tetra = {v0, v1, v2, v3};

    //        // if inverted then fix the orientation
    //        Vector3r v0v1 = v_rational[v1] - v_rational[v0];
    //        Vector3r v0v2 = v_rational[v2] - v_rational[v0];
    //        Vector3r v0v3 = v_rational[v3] - v_rational[v0];
    //        if ((v0v1.cross(v0v2)).dot(v0v3) < 0) {
    //            tetra = {v1, v0, v2, v3};
    //        }

    //        // push the tet to final queue;
    //        tets_final.push_back(tetra);

    //        std::set<size_t> local_f1 = {tetra[0], tetra[1], tetra[2]};
    //        std::set<size_t> local_f2 = {tetra[0], tetra[2], tetra[3]};
    //        std::set<size_t> local_f3 = {tetra[0], tetra[1], tetra[3]};
    //        std::set<size_t> local_f4 = {tetra[1], tetra[2], tetra[3]};

    //        // track surface     need to be fixed
    //        bool tet_face_on_input[4];
    //        for (auto f : polygon_cell) {
    //            std::set<size_t> f_vs = {
    //                polygon_faces[f][0],
    //                polygon_faces[f][1],
    //                polygon_faces[f][2]};

    //            int local_f_idx;

    //            // decide which face it is

    //            if (f_vs == local_f1) {
    //                local_f_idx = 0;
    //            } else if (f_vs == local_f2) {
    //                local_f_idx = 1;
    //            } else if (f_vs == local_f3) {
    //                local_f_idx = 2;
    //            } else {
    //                local_f_idx = 3;
    //            }

    //            tet_face_on_input[local_f_idx] = polygon_faces_on_input[f];
    //        }

    //        for (int k = 0; k < 4; k++) {
    //            tet_face_on_input_surface.push_back(tet_face_on_input[k]);
    //        }
    //        continue;
    //    }

    //    // compute centroid
    //    Vector3r centroid(0, 0, 0);
    //    for (auto v : polygon_vertices) {
    //        centroid = centroid + v_rational[v];
    //    }
    //    centroid = centroid / polygon_vertices.size();

    //    // trahedralize
    //    size_t centroid_idx = v_rational.size();
    //    v_rational.push_back(centroid);

    //    for (auto f : polygon_cell) {
    //        for (auto t : map_poly_to_tri_face[f]) {
    //            std::array<size_t, 4> tetra = {
    //                triangulated_faces[t][0],
    //                triangulated_faces[t][1],
    //                triangulated_faces[t][2],
    //                centroid_idx};
    //            // std::sort(tetra.begin(), tetra.end());
    //            // check inverted tet and fix
    //            Vector3r v0v1 = v_rational[tetra[1]] - v_rational[tetra[0]];
    //            Vector3r v0v2 = v_rational[tetra[2]] - v_rational[tetra[0]];
    //            Vector3r v0v3 = v_rational[tetra[3]] - v_rational[tetra[0]];
    //            if ((v0v1.cross(v0v2)).dot(v0v3) < 0) {
    //                tetra = {
    //                    triangulated_faces[t][1],
    //                    triangulated_faces[t][0],
    //                    triangulated_faces[t][2],
    //                    centroid_idx};
    //            }

    //            tets_final.push_back(tetra);
    //            tet_face_on_input_surface.push_back(triangulated_faces_on_input[t]);
    //            tet_face_on_input_surface.push_back(false);
    //            tet_face_on_input_surface.push_back(false);
    //            tet_face_on_input_surface.push_back(false);
    //        }
    //    }
    //}

    // std::cout << "polygon was tet num: " << was_tet_cnt << std::endl;
    // std::cout << "vertices final num: " << v_rational.size() << std::endl;
    // std::cout << "tets final num: " << tets_final.size() << std::endl;

    // std::cout << "track face size: " << tet_face_on_input_surface.size() << std::endl;

    // facets_after = triangulated_faces;
    // tets_after = tets_final;

    //// track vertices on input
    // is_v_on_input.reserve(v_rational.size());
    // for (int i = 0; i < v_rational.size(); i++) is_v_on_input.push_back(false);
    // for (int i = 0; i < triangulated_faces.size(); i++) {
    //     if (triangulated_faces_on_input[i]) {
    //         is_v_on_input[triangulated_faces[i][0]] = true;
    //         is_v_on_input[triangulated_faces[i][1]] = true;
    //         is_v_on_input[triangulated_faces[i][2]] = true;
    //     }
    // }

    // size_t on_surface_v_cnt = 0;
    // for (size_t i = 0; i < is_v_on_input.size(); i++) {
    //     if (is_v_on_input[i]) on_surface_v_cnt++;
    // }

    // std::cout << "v on surface vector size: " << is_v_on_input.size();
    // std::cout << "v on surface: " << on_surface_v_cnt << std::endl;

    // for (const auto& vids : tets_after) {
    //     Vector3r n = (v_rational[vids[1]] - v_rational[vids[0]])
    //                      .cross(v_rational[vids[2]] - v_rational[vids[0]]);
    //     Vector3r d = v_rational[vids[3]] - v_rational[vids[0]];
    //     auto res = n.dot(d);
    //     if (res <= 0) {
    //         logger().error("After insertion: Tet {} is inverted! res = {}", vids,
    //         res.to_double()); logger().error("v0 = {}",
    //         to_double(v_rational[vids[0]]).transpose()); logger().error("v1 = {}",
    //         to_double(v_rational[vids[1]]).transpose()); logger().error("v2 = {}",
    //         to_double(v_rational[vids[2]]).transpose()); logger().error("v3 = {}",
    //         to_double(v_rational[vids[3]]).transpose()); logger().error("res == 0 ? --> {}", res
    //         == 0);
    //     }
    // }
}

bool TetWildMesh::check_nondegenerate_tets()
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

void TetWildMesh::init_from_Volumeremesher(
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::array<size_t, 3>>& facets,
    const std::vector<bool>& is_v_on_input,
    const std::vector<std::array<size_t, 4>>& tets,
    const std::vector<bool>& tet_face_on_input_surface)
{
    init_with_isolated_vertices(v_rational.size(), tets);
    assert(check_mesh_connectivity_validity());

    m_vertex_attribute.resize(v_rational.size());
    m_tet_attribute.resize(tets.size());
    m_face_attribute.resize(tets.size() * 4);

    // A VolumeRemesher output vertex is either a direct (explicit) point -- whose
    // exact coordinate is already a double -- or an indirect (implicit) point that
    // is genuinely rational. A direct point rounds for free: its coordinate already
    // equals its double, so snapping it cannot invert any tet. Mark those rounded
    // here and remember which are direct, so the rounding pass below only does real
    // work (and can do it in parallel) for the indirect points.
    std::vector<char> is_direct_point(vert_capacity(), 0);
    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_pos = v_rational[i];
        // wmtk::logger().info("rational: {}", m_vertex_attribute[i].m_pos);
        m_vertex_attribute[i].m_posf = to_double(v_rational[i]);
        // wmtk::logger().info("double: {}", m_vertex_attribute[i].m_posf);
        const Vector3r& r = m_vertex_attribute[i].m_pos;
        const Vector3d& d = m_vertex_attribute[i].m_posf;
        const bool direct = (wmtk::Rational(d[0]) == r[0]) && (wmtk::Rational(d[1]) == r[1]) &&
                            (wmtk::Rational(d[2]) == r[2]);
        is_direct_point[i] = direct ? 1 : 0;
        m_vertex_attribute[i].m_is_rounded = direct;
    }

    // check here
    for (size_t i = 0; i < tet_face_on_input_surface.size(); i++) {
        // if (tet_face_on_input_surface[i]) m_face_attribute[i].m_is_surface_fs = 1;
        m_face_attribute[i].m_is_surface_fs = tet_face_on_input_surface[i];
    }

    const auto faces = get_faces();
    logger().info("#faces = {}", faces.size());

    wmtk::task_arena arena(std::max(1, NUM_THREADS));

    // mark surface vertices (parallel). Different faces write `true` to the shared
    // m_is_on_surface of a common vertex; atomic_ref makes those same-value writes
    // well-defined instead of a data race.
    arena.execute([&] {
        wmtk::parallel_for(
            wmtk::blocked_range<size_t>(0, faces.size()),
            [&](wmtk::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    const Tuple& f = faces[i];
                    if (m_face_attribute[f.fid(*this)].m_is_surface_fs != 1) continue;
                    const size_t v1 = f.vid(*this);
                    const size_t v2 = f.switch_vertex(*this).vid(*this);
                    const size_t v3 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
                    std::atomic_ref<bool>(m_vertex_attribute[v1].m_is_on_surface).store(true);
                    std::atomic_ref<bool>(m_vertex_attribute[v2].m_is_on_surface).store(true);
                    std::atomic_ref<bool>(m_vertex_attribute[v3].m_is_on_surface).store(true);
                }
            });
    });

    // track bounding box (parallel). The per-face exact-rational corner test is the
    // cost; on-bbox faces are rare, so each chunk collects (vertex, bbox-side) pairs
    // locally and merges once, and the per-vertex vectors are appended serially.
    {
        std::vector<std::pair<size_t, int>> bbox_vert_faces;
        std::mutex bbox_mutex;
        arena.execute([&] {
            wmtk::parallel_for(
                wmtk::blocked_range<size_t>(0, faces.size()),
                [&](wmtk::blocked_range<size_t> r) {
                    std::vector<std::pair<size_t, int>> local;
                    for (size_t i = r.begin(); i < r.end(); i++) {
                        auto vs = get_face_vertices(faces[i]);
                        std::array<size_t, 3> vids = {
                            {vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
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
                        m_face_attribute[faces[i].fid(*this)].m_is_bbox_fs = on_bbox;
                        for (size_t vid : vids) local.emplace_back(vid, on_bbox);
                    }
                    if (local.empty()) return;
                    std::lock_guard<std::mutex> lk(bbox_mutex);
                    bbox_vert_faces.insert(bbox_vert_faces.end(), local.begin(), local.end());
                });
        });
        for (const auto& [vid, on_bbox] : bbox_vert_faces)
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
    }

    for_each_vertex(
        [&](auto& v) { vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });

    // track open boundaries
    find_open_boundary();

    init_vertex_order();
    {
        std::array<size_t, 4> vo{{0, 0, 0, 0}};
        const auto vs = get_vertices();
        for (const Tuple& t : vs) {
            const size_t o = m_vertex_attribute.at(t.vid(*this)).m_order;
            assert(o < 4);
            ++vo[o];
        }
        assert(vo[0] + vo[1] + vo[2] + vo[3] == vs.size());
        logger().info("{} vertices with order counts (0,1,2,3): {}", vs.size(), vo);
    }

    // enable on for tests
    // debug codes
    // if (!check_nondegenerate_tets()) {
    //     std::cout << "find tet with <=0 volume!" << std::endl;
    // }

    // if (!check_mesh_connectivity_validity()) {
    //     std::cout << "invalid mesh connectivity!" << std::endl;
    // }

    // std::cout << "#edge_params: " << edge_params.size() << std::endl;


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


    // Round the indirect vertices in parallel. Rounding snaps m_pos to its double
    // and may not invert any incident tet, so it is not independent across adjacent
    // vertices and the old code did it serially (round() per vertex, an exact-
    // rational inversion check over each vertex's incident tets). Instead, snap all
    // indirect vertices at once and then revalidate/revert the few conflicts:
    //   1. snap every not-yet-rounded (indirect) vertex to its double (parallel);
    //   2. scan all tets (parallel) -- any inverted tet must contain an indirect
    //      vertex (a direct vertex never moves, so cannot cause an inversion), so
    //      revert those indirect vertices to their exact rational coordinate;
    //   3. repeat until no tet is inverted.
    // This terminates: every non-empty pass un-rounds at least one vertex, and the
    // all-rational mesh is valid. Direct points are skipped throughout.
    for_each_vertex([&](const Tuple& v) {
        const size_t i = v.vid(*this);
        if (!m_vertex_attribute[i].m_is_rounded) {
            m_vertex_attribute[i].m_pos << m_vertex_attribute[i].m_posf[0],
                m_vertex_attribute[i].m_posf[1], m_vertex_attribute[i].m_posf[2];
            m_vertex_attribute[i].m_is_rounded = true;
        }
    });

    while (true) {
        wmtk::concurrent_vector<size_t> to_revert;
        for_each_tetra([&](const Tuple& t) {
            if (is_inverted(t)) {
                for (const size_t vid : oriented_tet_vids(t)) {
                    if (!is_direct_point[vid] && m_vertex_attribute[vid].m_is_rounded)
                        to_revert.push_back(vid);
                }
            }
        });
        if (to_revert.empty()) break;
        for (const size_t vid : to_revert) {
            if (m_vertex_attribute[vid].m_is_rounded) {
                m_vertex_attribute[vid].m_pos = v_rational[vid];
                m_vertex_attribute[vid].m_is_rounded = false;
            }
        }
    }

    const auto vertices = get_vertices();
    size_t cnt_round_parallel = 0;
    for (const auto& v : vertices)
        if (m_vertex_attribute[v.vid(*this)].m_is_rounded) ++cnt_round_parallel;

    // Final serial sweep: the parallel batch reverts a vertex whenever it lies in
    // any inverted tet, which can over-revert (a vertex may round fine once its
    // neighbours are committed). Retry the leftovers one at a time -- round() is a
    // no-op for the already-rounded majority, so this only pays for the few that
    // remain, and recovers vertices the batch conservatively reverted.
    for (const auto& v : vertices) round(v);

    size_t cnt_round = 0;
    for (const auto& v : vertices)
        if (m_vertex_attribute[v.vid(*this)].m_is_rounded) ++cnt_round;
    wmtk::logger().info(
        "cnt_round {}/{} (parallel {}, serial recovered {})",
        cnt_round,
        vertices.size(),
        cnt_round_parallel,
        cnt_round - cnt_round_parallel);

    // init qualities
    auto& m = *this;
    for_each_tetra([&m](auto& t) { m.m_tet_attribute[t.tid(m)].m_quality = m.get_quality(t); });
}

// void init_from_file(std::string input_dir)
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
//     m_vertex_attribute.resize(v_num);
//     m_tet_attribute.resize(tets.size());
//     m_face_attribute.resize(tets.size() * 4);

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

void TetWildMesh::find_open_boundary()
{
    const auto fs = get_faces();
    const auto es = get_edges();
    std::vector<int> edge_on_open_boundary(6 * tet_capacity(), 0);

    // for open boundary envelope
    std::vector<Eigen::Vector3d> v_posf(vert_capacity());
    std::vector<Eigen::Vector2i> open_boundaries;

    for (size_t i = 0; i < vert_capacity(); i++) {
        v_posf[i] = m_vertex_attribute[i].m_posf;
    }

    wmtk::task_arena arena(std::max(1, NUM_THREADS));

    // count incident surface faces per edge (parallel; atomic_ref increments the shared array)
    arena.execute([&] {
        wmtk::parallel_for(
            wmtk::blocked_range<size_t>(0, fs.size()),
            [&](wmtk::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    const Tuple& f = fs[i];
                    if (!m_face_attribute[f.fid(*this)].m_is_surface_fs) continue;
                    const size_t eid1 = f.eid(*this);
                    const size_t eid2 = f.switch_edge(*this).eid(*this);
                    const size_t eid3 = f.switch_vertex(*this).switch_edge(*this).eid(*this);
                    std::atomic_ref<int>(edge_on_open_boundary[eid1])
                        .fetch_add(1, std::memory_order_relaxed);
                    std::atomic_ref<int>(edge_on_open_boundary[eid2])
                        .fetch_add(1, std::memory_order_relaxed);
                    std::atomic_ref<int>(edge_on_open_boundary[eid3])
                        .fetch_add(1, std::memory_order_relaxed);
                }
            });
    });

    // collect open-boundary edges (exactly one incident surface face), parallel with a
    // per-chunk merge. The collected order differs from serial but the set is identical
    // and the boundary envelope built from it is order-independent.
    {
        std::mutex ob_mutex;
        arena.execute([&] {
            wmtk::parallel_for(
                wmtk::blocked_range<size_t>(0, es.size()),
                [&](wmtk::blocked_range<size_t> r) {
                    std::vector<Eigen::Vector2i> local;
                    for (size_t i = r.begin(); i < r.end(); i++) {
                        const Tuple& e = es[i];
                        if (edge_on_open_boundary[e.eid(*this)] != 1) continue;
                        const size_t v1 = e.vid(*this);
                        const size_t v2 = e.switch_vertex(*this).vid(*this);
                        std::atomic_ref<bool>(m_vertex_attribute[v1].m_is_on_open_boundary)
                            .store(true);
                        std::atomic_ref<bool>(m_vertex_attribute[v2].m_is_on_open_boundary)
                            .store(true);
                        local.emplace_back(v1, v2);
                    }
                    if (local.empty()) return;
                    std::lock_guard<std::mutex> lk(ob_mutex);
                    open_boundaries.insert(open_boundaries.end(), local.begin(), local.end());
                });
        });
    }

    wmtk::logger().info("open boundary num: {}", open_boundaries.size());

    if (open_boundaries.size() == 0) {
        return;
    }

    // init open boundary envelope
    m_open_boundary_envelope.init(v_posf, open_boundaries, m_params.epsr * m_params.diag_l / 2.0);
}

bool TetWildMesh::is_open_boundary_edge(const Tuple& e)
{
    size_t v1 = e.vid(*this);
    size_t v2 = e.switch_vertex(*this).vid(*this);
    if (!m_vertex_attribute[v1].m_is_on_open_boundary ||
        !m_vertex_attribute[v2].m_is_on_open_boundary)
        return false;

    return !m_open_boundary_envelope.is_outside(
        std::array<Eigen::Vector3d, 2>{
            {m_vertex_attribute[v1].m_posf, m_vertex_attribute[v2].m_posf}});
}

bool TetWildMesh::is_open_boundary_edge(const std::array<size_t, 2>& e)
{
    size_t v1 = e[0];
    size_t v2 = e[1];
    if (!m_vertex_attribute[v1].m_is_on_open_boundary ||
        !m_vertex_attribute[v2].m_is_on_open_boundary)
        return false;

    return !m_open_boundary_envelope.is_outside(
        std::array<Eigen::Vector3d, 2>{
            {m_vertex_attribute[v1].m_posf, m_vertex_attribute[v2].m_posf}});
}

int TetWildMesh::orient3D(
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
// bool TetWildMesh::checkTrackedFaces(
//     std::vector<vol_rem::bigrational>& vol_coords,
//     const std::vector<double>& surf_coords,
//     std::vector<uint32_t>& facets,
//     std::vector<uint32_t>& facets_on_input,
//     const std::vector<uint32_t>& surf_tris)
// {
//     std::vector<uint32_t> fstart; // Vector containing the starting index of each face in 'facets'
//     for (uint32_t f_i = 0; f_i < facets.size(); f_i += (facets[f_i] + 1)) fstart.push_back(f_i);
//     vol_rem::bigrational v1[3], v2[3], v3[3], v4[3];
//     for (uint32_t ti : facets_on_input) // For each facet in the tracked surface
//     {
//         uint32_t start = fstart[ti];
//         uint32_t fnv = facets[start]; // num face vertices
//         const uint32_t* face_vrts = facets.data() + start + 1;
//         size_t i = 0;
//         for (; i < surf_tris.size(); i += 3) { // For each input triangular facet 'i'
//             v2[0] = surf_coords[surf_tris[i] * 3]; // Let v2,v3,v4 be the coordinates of its three
//                                                    // vertices
//             v2[1] = surf_coords[surf_tris[i] * 3 + 1];
//             v2[2] = surf_coords[surf_tris[i] * 3 + 2];
//             v3[0] = surf_coords[surf_tris[i + 1] * 3];
//             v3[1] = surf_coords[surf_tris[i + 1] * 3 + 1];
//             v3[2] = surf_coords[surf_tris[i + 1] * 3 + 2];
//             v4[0] = surf_coords[surf_tris[i + 2] * 3];
//             v4[1] = surf_coords[surf_tris[i + 2] * 3 + 1];
//             v4[2] = surf_coords[surf_tris[i + 2] * 3 + 2];
//             uint32_t v = 0;
//             for (; v < fnv; v++) { // For each vertex 'v' of 'ti'
//                 const uint32_t vid = face_vrts[v];
//                 v1[0] = vol_coords[vid * 3]; // Let v1 be v's coordinates
//                 v1[1] = vol_coords[vid * 3 + 1];
//                 v1[2] = vol_coords[vid * 3 + 2];
//                 if (orient3D(
//                         v1[0],
//                         v1[1],
//                         v1[2],
//                         v2[0],
//                         v2[1],
//                         v2[2],
//                         v3[0],
//                         v3[1],
//                         v3[2],
//                         v4[0],
//                         v4[1],
//                         v4[2]) != 0)
//                     break; // 'v' is not coplanar with triangular facet 'i'
//             }
//             if (v == fnv) break; // All vertices of 'ti' are coplanar with triangular facet 'i'
//         }
//         if (i == surf_tris.size())
//             return false; // 'ti' is not coplanar with any triangular facet in surf_tris
//     }
//     return true;
// }

int TetWildMesh::orient3D_wmtk_rational(
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
bool TetWildMesh::checkTrackedFaces_wmtk_rational(
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

} // namespace wmtk::components::tetwild