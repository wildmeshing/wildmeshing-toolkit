
#pragma once
#include <spdlog/logger.h>
#include <filesystem>
namespace wmtk {

void trimesh_topology_initialization(
    Eigen::Ref<const Mesh::RowVectors3l> F,
    Eigen::Ref<Mesh::RowVectors3l> FV,
    Eigen::Ref<Mesh::RowVectors3l> FE,
    Eigen::Ref<Mesh::RowVectors3l> FF,
    Eigen::Ref<Mesh::VectorXl> VF,
    Eigen::Ref<Mesh::VectorXl> EF);

}