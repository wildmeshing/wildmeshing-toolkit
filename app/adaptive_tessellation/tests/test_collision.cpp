#include <catch2/catch.hpp>

#include <igl/edges.h>
#include <ipc/ipc.hpp>

TEST_CASE("collision_free_stepsize 2D", "[ipc-toolkit]")
{
    Eigen::MatrixXd vertices;
    vertices.resize(4, 2);
    vertices << 0, 0, 1, 0, 0, 1, 1, 1;

    Eigen::MatrixXi faces;
    faces.resize(2, 3);
    faces << 0, 1, 2, 1, 3, 2;

    Eigen::MatrixXi edges;
    igl::edges(faces, edges);

    // create CollisionMesh
    ipc::CollisionMesh m(vertices, edges, faces);

    // perform collision
    Eigen::MatrixXd vertices_new = vertices;
    vertices_new(0, 0) = 0.5;

    // is_step_collision_free(m, vertices, vertices_new);
    double t = compute_collision_free_stepsize(m, vertices, vertices_new);
    REQUIRE(t == 1.0);

    vertices_new(0, 0) = 2.0;
    t = compute_collision_free_stepsize(m, vertices, vertices_new);
    REQUIRE(t < 1.0);
}

TEST_CASE("collision_free_stepsize 3D", "[ipc-toolkit]")
{
    Eigen::MatrixXd vertices;
    vertices.resize(6, 3);
    vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1;

    Eigen::MatrixXi faces;
    faces.resize(2, 3);
    faces << 0, 1, 2, 3, 4, 5;

    // get edges
    Eigen::MatrixXi edges;
    igl::edges(faces, edges);
    // create CollisionMesh
    ipc::CollisionMesh m(vertices, edges, faces);

    // perform collision
    Eigen::MatrixXd vertices_new = vertices;
    vertices_new(0, 2) = 0.5;

    // is_step_collision_free(m, vertices, vertices_new);
    double t = compute_collision_free_stepsize(m, vertices, vertices_new);
    REQUIRE(t == 1.0);

    vertices_new(0, 2) = 2.0;
    t = compute_collision_free_stepsize(m, vertices, vertices_new);
    REQUIRE(t < 1.0);
}