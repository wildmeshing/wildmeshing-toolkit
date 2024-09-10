

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
    auto [V, nvert] = vdat;
    auto [F, ntri] = fdat;

    std::cout << "here 1" << std::endl;


    std::cout << "here 2" << std::endl;


    std::cout << "here 3" << std::endl;

    plc.initFromVectors(V.data(), V.size(), F.data(), ntri, true);

    std::cout << "here 4" << std::endl;


    std::cout << "here 5" << std::endl;
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

std::shared_ptr<wmtk::TetMesh> convert_cdt_to_rational_wmtk_tetmesh(
    cdt::TetMesh& tin,
    std::vector<std::array<bool, 4>>& local_f_on_input,
    bool inner_only)
{
    MatrixX<int64_t> T;
    MatrixX<Rational> V;

    uint32_t ngnt = 0;
    for (uint32_t i = 0; i < tin.numTets(); ++i) {
        if (tin.mark_tetrahedra[i] == 2) {
            ngnt++;
        }
    }

    V.resize(tin.numVertices(), 3);

    if (inner_only) {
        T.resize(ngnt, 4);
    } else {
        T.resize(tin.countNonGhostTets(), 4);
    }


    for (uint32_t i = 0; i < tin.numVertices(); ++i) {
        cdt::bigrational c[3];
        tin.vertices[i]->getExactXYZCoordinates(c[0], c[1], c[2]);

#ifdef USE_GNU_GMP_CLASSES
        V(i, 0).init(c[0].get_mpq_t());
        V(i, 1).init(c[1].get_mpq_t());
        V(i, 2).init(c[2].get_mpq_t());
#else
        V(i, 0).init_from_binary(c[0].get_str());
        V(i, 1).init_from_binary(c[1].get_str());
        V(i, 2).init_from_binary(c[2].get_str());
#endif
    }

    int64_t row = 0;
    for (uint32_t i = 0; i < tin.numTets(); ++i) {
        assert(row < T.rows());
        if (tin.mark_tetrahedra[i] == 2) {
            T(row, 0) = tin.tet_node[i * 4];
            T(row, 1) = tin.tet_node[i * 4 + 1];
            T(row, 2) = tin.tet_node[i * 4 + 2];
            T(row, 3) = tin.tet_node[i * 4 + 3];

            // mark surfaces
            // assuming opposite face to node
            std::array<bool, 4> on_input = {{false, false, false, false}};
            for (uint64_t j = i; j < i + 4; ++j) {
                if (tin.mark_tetrahedra[tin.tet_neigh[j] >> 2] != tin.mark_tetrahedra[j >> 2]) {
                    on_input[j] = true;
                }
            }

            local_f_on_input.push_back(on_input);
        }
        row++;
    }

    if (!inner_only) {
        for (uint32_t i = 0; i < tin.numTets(); ++i) {
            assert(row < T.rows());
            if (!tin.isGhost(i) && tin.mark_tetrahedra[i] != 2) {
                T(row, 0) = tin.tet_node[i * 4];
                T(row, 1) = tin.tet_node[i * 4 + 1];
                T(row, 2) = tin.tet_node[i * 4 + 2];
                T(row, 3) = tin.tet_node[i * 4 + 3];

                // mark surfaces
                // assuming opposite face to node
                std::array<bool, 4> on_input = {{false, false, false, false}};
                for (uint64_t j = i; j < i + 4; ++j) {
                    if (tin.mark_tetrahedra[tin.tet_neigh[j] >> 2] != tin.mark_tetrahedra[j >> 2]) {
                        on_input[j] = true;
                    }
                }

                local_f_on_input.push_back(on_input);
            }
            row++;
        }
    }

    assert(local_f_on_input.size() == T.rows());

    std::shared_ptr<wmtk::TetMesh> m = std::make_shared<wmtk::TetMesh>();
    m->initialize(T);

    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *m);

    return m;
}

std::shared_ptr<wmtk::TetMesh> CDT_internal(
    const wmtk::TriMesh& m,
    std::vector<std::array<bool, 4>>& local_f_on_input,
    bool inner_only)
{
    cdt::inputPLC plc;

    convert_trimesh_to_input_plc(m, plc);

    cdt::TetMesh* tin = createSteinerCDT(plc, true, false);

    return convert_cdt_to_rational_wmtk_tetmesh(*tin, local_f_on_input, inner_only);
}

} // namespace wmtk::components::internal
