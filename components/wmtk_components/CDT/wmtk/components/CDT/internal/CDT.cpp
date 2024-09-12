

//
#include "CDT.hpp"
#include <wmtk/utils/mesh_utils.hpp>
#include "get_vf.hpp"


#include <CDT/PLC.h>
#include <CDT/delaunay.h>
#include <CDT/inputPLC.h>


namespace wmtk::components::internal {

void convert_trimesh_to_input_plc(const TriMesh& trimesh, cdt::inputPLC& plc)
{
    // return;

    auto [vdat, fdat] = get_vf(trimesh);
    auto [V, npts] = vdat;
    auto [F, ntri] = fdat;

    plc.initFromVectors(V.data(), npts, F.data(), ntri, true);
}

cdt::TetMesh* createSteinerCDT(cdt::inputPLC& plc, bool bbox, bool snap)
{
    if (bbox) {
        plc.addBoundingBoxVertices();
    }

    cdt::TetMesh* tin = new cdt::TetMesh;
    tin->init_vertices(plc.coordinates.data(), plc.numVertices());
    tin->tetrahedrize();

    // copied from cdt/main.cpp
    // Build a structured PLC linked to the Delaunay tetrahedrization
    cdt::PLCx Steiner_plc(*tin, plc.triangle_vertices.data(), plc.numTriangles());

    // Recover segments by inserting Steiner points in both the PLC and the
    // tetrahedrization
    Steiner_plc.segmentRecovery_HSi(true);

    // Recover PLC faces by locally remeshing the tetrahedrization
    bool sisMethodWorks = Steiner_plc.faceRecovery(true);

    // Mark the tets which are bounded by the PLC.
    // If the PLC is not a valid polyhedron (i.e. it has odd-valency edges)
    // all the tets but the ghosts are marked as "internal".
    uint32_t num_inner_tets = (uint32_t)Steiner_plc.markInnerTets();

    // try to round to floating points
    // should be false if use for rational insertion
    if (snap) {
        if (!tin->optimizeNearDegenerateTets(true)) {
            wmtk::logger().error("Could not force FP representability.");
        }
    }

    return tin;
}

void convert_cdt_to_stl(
    cdt::TetMesh& tin,
    std::vector<std::array<bool, 4>>& local_f_on_input,
    std::vector<std::array<int64_t, 4>>& T_final,
    std::vector<std::array<Rational, 3>>& V_final,
    bool inner_only)
{
    std::vector<std::array<int64_t, 4>> T;
    std::vector<std::array<Rational, 3>> V;

    uint32_t ngnt = 0;
    for (uint32_t i = 0; i < tin.numTets(); ++i) {
        if (tin.mark_tetrahedra[i] == 2) {
            ngnt++;
        }
    }

    for (uint32_t i = 0; i < tin.numVertices(); ++i) {
        cdt::bigrational c[3];
        tin.vertices[i]->getExactXYZCoordinates(c[0], c[1], c[2]);

        std::array<Rational, 3> v;

#ifdef USE_GNU_GMP_CLASSES
        v[0].init(c[0].get_mpq_t());
        v[1].init(c[1].get_mpq_t());
        v[2].init(c[2].get_mpq_t());
#else
        v[0].init_from_binary(c[0].get_str());
        v[1].init_from_binary(c[1].get_str());
        v[2].init_from_binary(c[2].get_str());
#endif
        V.push_back(v);
    }

    int64_t row = 0;
    for (uint32_t i = 0; i < tin.numTets(); ++i) {
        if (tin.mark_tetrahedra[i] == 2) {
            T.push_back(
                {{tin.tet_node[i * 4],
                  tin.tet_node[i * 4 + 1],
                  tin.tet_node[i * 4 + 2],
                  tin.tet_node[i * 4 + 3]}});

            // mark surfaces
            // assuming opposite face to node
            std::array<bool, 4> on_input = {{false, false, false, false}};
            for (uint64_t j = i; j < i + 4; ++j) {
                if (tin.mark_tetrahedra[tin.tet_neigh[j] >> 2] != tin.mark_tetrahedra[j >> 2]) {
                    on_input[j - i] = true;
                }
            }

            local_f_on_input.push_back(on_input);

            row++;
        }
    }

    if (!inner_only) {
        for (uint32_t i = 0; i < tin.numTets(); ++i) {
            if (!tin.isGhost(i) && tin.mark_tetrahedra[i] != 2) {
                T.push_back(
                    {{tin.tet_node[i * 4],
                      tin.tet_node[i * 4 + 1],
                      tin.tet_node[i * 4 + 2],
                      tin.tet_node[i * 4 + 3]}});

                // mark surfaces
                // assuming opposite face to node
                std::array<bool, 4> on_input = {{false, false, false, false}};
                for (uint64_t j = i; j < i + 4; ++j) {
                    if (tin.mark_tetrahedra[tin.tet_neigh[j] >> 2] != tin.mark_tetrahedra[j >> 2]) {
                        on_input[j] = true;
                    }
                }

                local_f_on_input.push_back(on_input);

                row++;
            }
        }
    }

    // remove unused vertices
    std::vector<int> v_is_used_in_tet(V.size(), 0);
    for (int64_t i = 0; i < T.size(); ++i) {
        for (int64_t j = 0; j < 4; ++j) {
            v_is_used_in_tet[T[i][j]] = 1;
        }
    }

    int64_t v_used_cnt = std::count(v_is_used_in_tet.begin(), v_is_used_in_tet.end(), 1);

    std::map<int64_t, int64_t> v_map;

    for (int64_t i = 0; i < V.size(); ++i) {
        if (v_is_used_in_tet[i]) {
            v_map[i] = V_final.size();
            V_final.push_back(V[i]);
        }
    }

    T_final = T;

    for (int64_t i = 0; i < T.size(); ++i) {
        for (int64_t j = 0; j < 4; ++j) {
            T_final[i][j] = v_map[T[i][j]];
        }
    }
}

std::shared_ptr<wmtk::TetMesh> CDT_internal(
    const wmtk::TriMesh& m,
    std::vector<std::array<bool, 4>>& local_f_on_input,
    bool inner_only)
{
    cdt::inputPLC plc;

    convert_trimesh_to_input_plc(m, plc);

    cdt::TetMesh* tin = createSteinerCDT(plc, true, false);


    std::vector<std::array<int64_t, 4>> T_final;
    std::vector<std::array<Rational, 3>> V_final;

    convert_cdt_to_stl(*tin, local_f_on_input, T_final, V_final, inner_only);

    MatrixX<int64_t> T;
    MatrixX<Rational> V;

    T.resize(T_final.size(), 4);
    V.resize(V_final.size(), 3);

    for (int64_t i = 0; i < T_final.size(); ++i) {
        T(i, 0) = T_final[i][0];
        T(i, 1) = T_final[i][1];
        T(i, 2) = T_final[i][2];
        T(i, 3) = T_final[i][3];
    }

    for (int64_t i = 0; i < V_final.size(); ++i) {
        V(i, 0) = V_final[i][0];
        V(i, 1) = V_final[i][1];
        V(i, 2) = V_final[i][2];
    }

    std::shared_ptr<wmtk::TetMesh> tm = std::make_shared<wmtk::TetMesh>();
    tm->initialize(T);

    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *tm);

    return tm;
}

} // namespace wmtk::components::internal
