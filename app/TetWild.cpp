//
// Created by Yixin Hu on 11/3/21.
//

#include "TetWild.h"
#include "external/MshSaver.h"

#include <wmtk/AMIPS.h>
#include "Logger.hpp"

#include <igl/predicates/predicates.h>

bool tetwild::TetWild::is_inverted(const Tuple& loc)
{
    std::array<Vector3f, 4> ps;
    auto its = loc.oriented_tet_vertices(*this);
    for (int j = 0; j < 4; j++) {
        ps[j] = m_vertex_attribute[its[j].vid()].m_posf;
    }

    //
    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(ps[0], ps[1], ps[2], ps[3]);
    Scalar result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result <= 0) return true;
    return false;
}

double tetwild::TetWild::get_quality(const Tuple& loc)
{
    std::array<Vector3f, 4> ps;
    auto its = loc.oriented_tet_vertices(*this);
    for (int j = 0; j < 4; j++) {
        ps[j] = m_vertex_attribute[its[j].vid()].m_posf;
    }

    std::array<double, 12> T;
    for (int j = 0; j < 3; j++) {
        T[0 * 3 + j] = ps[0][j];
        T[1 * 3 + j] = ps[1][j];
        T[2 * 3 + j] = ps[2][j];
        T[3 * 3 + j] = ps[3][j];
    }

    double energy = wmtk::AMIPS_energy(T);
    if (std::isinf(energy) || std::isnan(energy) || energy < 3 - 1e-3) return MAX_ENERGY;
    return energy;
}


bool tetwild::TetWild::vertex_invariant(const Tuple& t)
{
    return true;
}

bool tetwild::TetWild::tetrahedron_invariant(const Tuple& t)
{
    return true;
}

void tetwild::TetWild::output_mesh(std::string file)
{
    PyMesh::MshSaver mSaver(file, true);

    Eigen::VectorXd V_flat(3 * m_vertex_attribute.size());
    for (int i = 0; i < m_vertex_attribute.size(); i++) {
        for (int j = 0; j < 3; j++)
            V_flat(3 * i + j) = m_vertex_attribute[i].m_posf[j];
    }

    Eigen::VectorXi T_flat(4 * n_tets());
    for (int i = 0; i < n_tets(); i++) {
        Tuple loc = tuple_from_tet(i);
        auto vs = oriented_tet_vertices(loc);
        for (int j = 0; j < 4; j++) {
            T_flat(4 * i + j) = vs[j].vid();
        }
    }

    mSaver.save_mesh(V_flat, T_flat, 3, mSaver.TET);
}
