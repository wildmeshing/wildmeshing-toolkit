#include <igl/write_triangle_mesh.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>

#include <catch2/catch.hpp>
#include <memory>
#include "spdlog/common.h"

#include <igl/read_triangle_mesh.h>

#include <metis.h>

using namespace wmtk;

TEST_CASE("partition-mesh", "[test_partition]")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    std::string input_path = WMT_DATA_DIR "/max-planck.obj";
    igl::read_triangle_mesh(input_path, V, F);

    auto partitioned_v = partition_mesh_vertices(F, 10);
    REQUIRE(partitioned_v.size() == V.rows());
    REQUIRE(partitioned_v.maxCoeff() == 9);
}


#ifdef WMTK_WITH_PARMETIS
#include <mpi.h>
#include <parmetis.h>


namespace wmtk {

Eigen::Matrix<metis_index_t, Eigen::Dynamic, 1> parallel_partition_mesh_vertices_raw(
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
    logger().info("Parallel Partition, num parts: {}", num_partitions);

    auto elmdist = std::array<idx_t, 2>{{0, num_nodes}};
    auto zero = idx_t(0);
    auto one = idx_t(1);
    auto comm = MPI_COMM_WORLD;
    real_t ubvec = 1.05;
    idx_t wgtflag = 0;
    idx_t numflag = 0;
    idx_t ncon = 1;
    idx_t ncommonnodes = 2;
    idx_t options[3] = {0, 0, 0};
    std::vector<real_t> tpwgts(ncon*num_partitions, 1.0/num_partitions);

    auto provided = 0;
    MPI_Init_thread(nullptr, nullptr,MPI_THREAD_FUNNELED, &provided);
    auto err = ParMETIS_V3_PartMeshKway(
        &elmdist[0], // distribution on processors
        e_ptr,
        e_ind,
        nullptr, // elmwgt
        &wgtflag, // wgtflag, 0
        &numflag, // numflag, 0
        &ncon, // ncon
        &ncommonnodes, // ncomonnodes
        &num_partitions, // nparts
        &tpwgts[0], // tpwgts
        &ubvec, // ubvec
        options, // options
        e_part.get(),
        n_part.get(),
        &comm);
    MPI_Finalize();

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


TEST_CASE("parallel-partition-mesh", "[test_partition]")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXd facets;
    std::string input_path = WMT_DATA_DIR "/max-planck.obj";
    igl::read_triangle_mesh(input_path, V, facets);

    constexpr int num_partitions = 10;

    const auto num_elems = static_cast<metis_index_t>(facets.rows());
    const auto num_nodes = static_cast<metis_index_t>(facets.maxCoeff() + 1);
    auto res = convert_index_buffer(facets);
    auto partitioned_v = parallel_partition_mesh_vertices_raw(
        num_elems,
        num_nodes,
        res.first.get(),
        res.second.get(),
        num_partitions);
    REQUIRE(partitioned_v.size() == V.rows());
    REQUIRE(partitioned_v.maxCoeff() == 9);
    CHECK((partitioned_v.array()==1).count() > 300);
    CHECK((partitioned_v.array()==0).count() > 300);
}

#endif