#include <Eigen/Core>

#include <memory>

namespace wmtk {

/// Index type used by METIS
using metis_index_t = int32_t;

///
/// Converts a list of facet indices into flat buffers suitable to be used by METIS.
///
/// @param[in]  facets    #F x k array of face indices.
///
/// @tparam     DerivedF  Facet array type.
///
/// @return     A pair of buffers (eptr, eind) following the data structure explained in Section 5.6
///             of the [METIS manual](http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/manual.pdf).
///
template <typename DerivedF>
std::pair<std::unique_ptr<metis_index_t[]>, std::unique_ptr<metis_index_t[]>> convert_index_buffer(
    const Eigen::MatrixBase<DerivedF>& facets)
{
    const auto num_elems = static_cast<metis_index_t>(facets.rows());
    const auto elem_size = static_cast<metis_index_t>(facets.cols());

    auto e_ptr = std::unique_ptr<metis_index_t[]>(new metis_index_t[num_elems + 1]);
    auto e_ind = std::unique_ptr<metis_index_t[]>(new metis_index_t[num_elems * elem_size]);
    for (metis_index_t f = 0; f < num_elems; ++f) {
        e_ptr[f] = f * elem_size;
        for (metis_index_t lv = 0; lv < elem_size; ++lv) {
            e_ind[f * elem_size + lv] = facets(f, lv);
        }
    }
    e_ptr[num_elems] = num_elems * elem_size;
    return std::make_pair(std::move(e_ptr), std::move(e_ind));
}

///
/// Low-level function wrapping partitioning call to METIS.
///
/// @param[in]     num_elems       Number of elements in the mesh.
/// @param[in]     num_nodes       Number of nodes in the mesh.
/// @param[in,out] e_ptr           Offset array indicating where each element index starts.
/// @param[in,out] e_ind           Flat array of vertex indices for each element.
/// @param[in]     num_partitions  Number of partitions to produce.
///
/// @return        #V x 1 array of partition ids.
///
Eigen::Matrix<metis_index_t, Eigen::Dynamic, 1> partition_mesh_vertices_raw(
    metis_index_t num_elems,
    metis_index_t num_nodes,
    metis_index_t* e_ptr,
    metis_index_t* e_ind,
    metis_index_t num_partitions);

///
/// Partition mesh vertices into num_partitions using METIS.
///
/// @param[in]  facets          #F x k array of facet indices.
/// @param[in]  num_partitions  Number of partitions to produce.
///
/// @tparam     DerivedF        Type of facet array.
///
/// @return     #V x 1 array of partition ids.
///
template <typename DerivedF>
Eigen::Matrix<metis_index_t, Eigen::Dynamic, 1> partition_mesh_vertices(
    const Eigen::MatrixBase<DerivedF>& facets,
    metis_index_t num_partitions)
{
    const auto num_elems = static_cast<metis_index_t>(facets.rows());
    const auto num_nodes = static_cast<metis_index_t>(facets.maxCoeff() + 1);
    auto res = convert_index_buffer(facets);
    return partition_mesh_vertices_raw(
        num_elems,
        num_nodes,
        res.first.get(),
        res.second.get(),
        num_partitions);
}

} // namespace wmtk
