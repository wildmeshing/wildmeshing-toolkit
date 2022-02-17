#include <wmtk/utils/Partitioning.h>
#include <wmtk/utils/Logger.hpp>

#include <metis.h>
#ifdef WMTK_WITH_PARMETIS
#include <parmetis.h>
#endif

#include <type_traits>

namespace wmtk {

Eigen::Matrix<metis_index_t, Eigen::Dynamic, 1> partition_mesh_vertices_raw(
    metis_index_t num_elems,
    metis_index_t num_nodes,
    metis_index_t* e_ptr,
    metis_index_t* e_ind,
    metis_index_t num_partitions)
{
    static_assert(std::is_same<::wmtk::metis_index_t, idx_t>::value, "Index types don't match");

    // Sanity check
    Eigen::Matrix<metis_index_t, Eigen::Dynamic, 1> partitions(num_nodes);
    if (num_partitions <= 1) {
        logger().debug("<= 1 partition was requested, skipping partitioning.");
        partitions.setZero();
        return partitions;
    }

    // Outputs
    idx_t objval = 0;
    auto e_part = std::unique_ptr<idx_t[]>(new idx_t[num_elems]);
    auto n_part = std::unique_ptr<idx_t[]>(new idx_t[num_nodes]);
    logger().info("Num parts: {}", num_partitions);

    // Do the job. Lots of options, stuff is explained here:
    // http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/manual.pdf
    auto err = METIS_PartMeshNodal(
        &num_elems,
        &num_nodes,
        e_ptr,
        e_ind,
        nullptr, // vwgt
        nullptr, // vsize
        &num_partitions,
        nullptr, // tpwgts
        nullptr, // options
        &objval,
        e_part.get(),
        n_part.get());

    // Error handling
    std::string message;
    switch (err) {
    case METIS_OK:
        logger().debug(
            "[partitioning] Computed {} partitions with total score of {}",
            num_partitions,
            objval);
        break;
    case METIS_ERROR_INPUT: message = "[partitioning] Invalid input."; break;
    case METIS_ERROR_MEMORY: message = "[partitioning] Ran out of memory."; break;
    case METIS_ERROR:
    default: message = "[partitioning] Ran out of memory."; break;
    }
    if (!message.empty()) {
        logger().error("{}", message);
        throw std::runtime_error(message);
    }

    // Convert back output
    for (idx_t i = 0; i < num_nodes; ++i) {
        partitions(i) = n_part[i];
    }
    return partitions;
}

} // namespace wmtk
