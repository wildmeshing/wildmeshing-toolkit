
#pragma once

#include <Eigen/Core>
#include <array>
#include <vector>

namespace wmtk {

/**
 * @brief This table contains all possible configurations for curring a tetrahedra with a plane. 
 * The script for generating the table is avilable at https://github.com/wildmeshing/fTetWild 
 *
 */
class CutTable
{
public:
    static const std::vector<std::vector<Eigen::Vector4i>>& get_tet_confs(const int idx);
    static const std::vector<std::vector<Eigen::Vector2i>>& get_diag_confs(const int idx);
    static const std::vector<std::vector<std::array<bool, 4>>>& get_surface_conf(const int idx);
    static const std::vector<std::vector<Eigen::Vector4i>>& get_face_id_conf(const int idx);
    static inline const std::vector<Eigen::Vector4i>& get_tet_conf(const int idx, const int cfg)
    {
        return get_tet_confs(idx)[cfg];
    }
    static inline const std::vector<std::array<bool, 4>>& get_surface_conf(
        const int idx,
        const int cfg)
    {
        return get_surface_conf(idx)[cfg];
    }
    static inline const std::vector<Eigen::Vector4i>& get_face_id_conf(const int idx, const int cfg)
    {
        return get_face_id_conf(idx)[cfg];
    }
};
} // namespace wmtk
