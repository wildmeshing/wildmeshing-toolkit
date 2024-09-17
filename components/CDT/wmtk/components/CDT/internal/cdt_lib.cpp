#include "cdt_lib.hpp"

#include <CDT/PLC.h>
#include <CDT/delaunay.h>
#include <CDT/inputPLC.h>
#include <map>

namespace cdt_lib {

void convert_trimesh_to_input_plc(
    const std::vector<double>& V,
    const uint32_t npts,
    const std::vector<uint32_t>& F,
    const uint32_t ntri,
    cdt::inputPLC& plc)
{
    plc.initFromVectors(
        const_cast<double*>(V.data()),
        npts,
        const_cast<uint32_t*>(F.data()),
        ntri,
        true);
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

    return tin;
}

void convert_cdt_to_stl(
    cdt::TetMesh& tin,
    std::vector<std::array<bool, 4>>& local_f_on_input,
    std::vector<std::array<int64_t, 4>>& T_final,
    std::vector<std::array<std::string, 3>>& V_final,
    bool inner_only)
{
    std::vector<std::array<int64_t, 4>> T;
    std::vector<std::array<std::string, 3>> V;

    uint32_t ngnt = 0;
    for (uint32_t i = 0; i < tin.numTets(); ++i) {
        if (tin.mark_tetrahedra[i] == 2) {
            ngnt++;
        }
    }

    for (uint32_t i = 0; i < tin.numVertices(); ++i) {
        cdt::bigrational c[3];
        tin.vertices[i]->getExactXYZCoordinates(c[0], c[1], c[2]);

        std::array<std::string, 3> v;

#ifdef USE_GNU_GMP_CLASSES // TODO
        v[0].init(c[0].get_mpq_t());
        v[1].init(c[1].get_mpq_t());
        v[2].init(c[2].get_mpq_t());
#else
        v[0] = c[0].get_str();
        v[1] = c[1].get_str();
        v[2] = c[2].get_str();
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

void cdt_to_string(
    const std::vector<double>& V,
    const uint32_t npts,
    const std::vector<uint32_t>& F,
    const uint32_t ntri,
    std::vector<std::array<bool, 4>>& local_f_on_input,
    std::vector<std::array<int64_t, 4>>& T_final,
    std::vector<std::array<std::string, 3>>& V_final,
    bool inner_only)
{
    cdt::inputPLC plc;

    convert_trimesh_to_input_plc(V, npts, F, ntri, plc);

    cdt::TetMesh* tin = createSteinerCDT(plc, true, false);
    convert_cdt_to_stl(*tin, local_f_on_input, T_final, V_final, inner_only);
}
} // namespace cdt_lib