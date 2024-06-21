#pragma once

#include <map>
#include <wmtk/TriMesh.hpp>

#include <filesystem>
namespace wmtk::components::internal {
struct VectorComparer
{
    bool operator()(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
    {
        return std::lexicographical_compare(
            a.data(),
            a.data() + a.size(),
            b.data(),
            b.data() + b.size());
    }
};

void extract_triangle_soup_from_image(
    std::string output_path,
    std::string filename,
    double delta_x,
    unsigned int max_level);

void read_array_data(
    std::vector<std::vector<std::vector<unsigned int>>>& data,
    const std::string& filename);

void octree_add_points(Eigen::MatrixXd& V, unsigned int max_level);

void octree_add_points(
    std::map<Eigen::Vector3d, bool, VectorComparer>& record,
    const std::vector<Eigen::Vector3d>& sub_points,
    unsigned int itr,
    double min_x,
    double min_y,
    double min_z,
    double max_x,
    double max_y,
    double max_z);

bool is_int(double v);

void readGmsh(
    const std::string& filename,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<Eigen::Vector4<unsigned int>>& tetrahedra);

void gmsh2hdf_tag(std::string volumetric_file, std::string gmsh_file, std::string output_file);
} // namespace wmtk::components::internal
