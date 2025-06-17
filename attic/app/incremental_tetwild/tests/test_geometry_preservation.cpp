#pragma once
#include <IncrementalTetWild.h>
#include <common.h>
#include <wmtk/TetMesh.h>

// #include <igl/copyleft/cgal/remesh_intersections.h>
#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/io.hpp>


#include "spdlog/common.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

using namespace wmtk;
using namespace tetwild;

TEST_CASE("jacobian_test", "[geometry preservation]")
{
    Vector3d o(1, 0, 1);
    Vector3d x(0, 1, 0);
    Vector3d y(1, 0, 0);
    Vector3d old_pos(2, 2, 1);
    std::array<double, 12> T = {{2, 2, 1, 2, 4, 2, -2, 2, 2, 2, 2, -2}};
    Vector2d uv((old_pos - o).dot(x), (old_pos - o).dot(y));
    std::cout << uv << std::endl;
    // param function
    auto param = [&o, &x, &y](const Vector2d& uv) { return o + uv[0] * x + uv[1] * y; };
    // jacobian with chainrule
    auto jac = [&x, &y](const std::array<double, 12>& T, Eigen::Vector2d& result) {
        Eigen::Matrix<double, 3, 2> jac_param;
        jac_param.col(0) = x;
        jac_param.col(1) = y;

        Vector3d jac_amips;
        wmtk::AMIPS_jacobian(T, jac_amips);

        std::cout << jac_amips << std::endl;
        std::cout << jac_param << std::endl;

        result = jac_amips.transpose() * jac_param;
    };

    // FD gradient
    double eps = 1e-4;
    auto du = uv;
    du[0] += eps; // du = uv; du[0] += eps;
    auto duT = T; // duT = T;

    for (auto j = 0; j < 3; j++) {
        duT[j] = param(du)[j]; // only filling the front point x,y,z.
    }
    double dufd = (wmtk::AMIPS_energy(duT) - wmtk::AMIPS_energy(T)) / eps;

    auto dv = uv;
    dv[1] += eps;
    auto dvT = T;

    for (auto j = 0; j < 3; j++) {
        dvT[j] = param(dv)[j]; // only filling the front point x,y,z.
    }
    double dvfd = (wmtk::AMIPS_energy(dvT) - wmtk::AMIPS_energy(T)) / eps;

    Vector2d J;
    jac(T, J);
    std::cout << "AMIPS(T): " << wmtk::AMIPS_energy(T) << std::endl;
    std::cout << "AMIPS(duT): " << wmtk::AMIPS_energy(duT) << std::endl;
    std::cout << "AMIPS(dvT): " << wmtk::AMIPS_energy(dvT) << std::endl;
    std::cout << "J[0]: " << J[0] << " dufd: " << dufd << std::endl;
    std::cout << "J[1]: " << J[1] << " dvfd: " << dvfd << std::endl;

    // end of FD

    // hessian with chainrule
    auto hessian = [&x, &y](const std::array<double, 12>& T, Eigen::Matrix2d& result) {
        Eigen::Matrix<double, 3, 2> jac_param;
        jac_param.col(0) = x;
        jac_param.col(1) = y;

        Eigen::Matrix3d hessian_amips;
        wmtk::AMIPS_hessian(T, hessian_amips);

        result = jac_param.transpose() * hessian_amips * jac_param;
    };

    du = uv;
    du[0] += eps;
    duT = T;

    for (auto j = 0; j < 3; j++) {
        duT[j] = param(du)[j]; // only filling the front point x,y,z.
    }
    Vector2d jacdu, jacn;
    jac(duT, jacdu);
    jac(T, jacn);
    Vector2d dufd2 = (jacdu - jacn) / eps;

    dv = uv;
    dv[1] += eps;
    dvT = T;
    for (auto j = 0; j < 3; j++) {
        dvT[j] = param(dv)[j]; // only filling the front point x,y,z.
    }
    Vector2d jacdv2, jacn2;
    jac(dvT, jacdv2);
    jac(T, jacn2);
    Vector2d dvfd2 = (jacdv2 - jacn2) / eps;

    Eigen::Matrix2d H;
    hessian(T, H);
    std::cout << "H[0]: " << H.col(0) << std::endl << " hufd: " << dufd2 << std::endl;
    std::cout << "H[1]: " << H.col(1) << std::endl << " hvfd: " << dvfd2 << std::endl;
    //     J.col(0) == dufd; J.col(1) == dvfd;
}

TEST_CASE("jacobian_test_edge", "[geometry preservation]")
{
    Vector3d o(1, 0, 1);
    Vector3d d(1, 2, 0);
    d = d.normalized();
    Vector3d old_pos(2, 2, 1);
    std::array<double, 12> T = {{2, 2, 1, 2, 4, 2, -2, 2, 2, 2, 2, -2}};
    double t = (old_pos - o).dot(d);
    std::cout << t << std::endl;
    // param function
    auto param = [&o, &d](double t) { return o + t * d; };
    // jacobian with chainrule
    auto jac = [&d](const std::array<double, 12>& T) {
        Vector3d jac_param;
        jac_param = d;

        Vector3d jac_amips;
        wmtk::AMIPS_jacobian(T, jac_amips);

        double result = jac_amips.dot(jac_param);
        return result;
    };

    // FD gradient
    double eps = 1e-4;
    auto du = t;
    du += eps; // du = uv; du[0] += eps;
    auto duT = T; // duT = T;

    for (auto j = 0; j < 3; j++) {
        duT[j] = param(du)[j]; // only filling the front point x,y,z.
    }
    double dufd = (wmtk::AMIPS_energy(duT) - wmtk::AMIPS_energy(T)) / eps;

    double J;
    J = jac(T);
    std::cout << "AMIPS(T): " << wmtk::AMIPS_energy(T) << std::endl;
    std::cout << "AMIPS(duT): " << wmtk::AMIPS_energy(duT) << std::endl;
    std::cout << "J: " << J << " dufd: " << dufd << std::endl;

    // end of FD

    // hessian with chainrule
    auto hessian = [&d](const std::array<double, 12>& T) {
        Vector3d jac_param;
        jac_param = d;

        Eigen::Matrix3d hessian_amips;
        wmtk::AMIPS_hessian(T, hessian_amips);

        double result = jac_param.transpose() * hessian_amips * jac_param;
        return result;
    };

    du = t;
    du += eps;
    duT = T;

    for (auto j = 0; j < 3; j++) {
        duT[j] = param(du)[j]; // only filling the front point x,y,z.
    }
    double jacdu, jacn;
    jacdu = jac(duT);
    jacn = jac(T);
    double dufd2 = (jacdu - jacn) / eps;

    double H;
    H = hessian(T);
    std::cout << "H: " << H << std::endl << " hufd: " << dufd2 << std::endl;
    //     J.col(0) == dufd; J.col(1) == dvfd;
}
