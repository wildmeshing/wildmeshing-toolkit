
#include <wmtk/TriMesh.h>
#include <wmtk/utils/OperationLogger.h>
#include <wmtk/utils/OperationReplayer.h>
#include <wmtk/ExecutionScheduler.hpp>

#include <wmtk/utils/OperationRecordingDataTypes.hpp>

template <>
HighFive::DataType HighFive::create_datatype<wmtk::AttributeChanges>();
template <>
HighFive::DataType HighFive::create_datatype<wmtk::TriMeshOperationData>();


using namespace wmtk;
OperationReplayer::OperationReplayer(TriMesh& m, const OperationLogger& logger_)
    : mesh(m)
    , logger(logger_)
{}


size_t OperationReplayer::operation_count() const
{
    return logger.operation_count();
}


size_t OperationReplayer::play(int step_count)
{
    size_t to;
    if (step_count < 0) {
        if (current_index > size_t(-step_count)) {
            to = current_index + step_count;
        } else {
            to = 0;
        }
    } else {
        to = current_index + step_count;
    }
    return play_to(to);
}

size_t OperationReplayer::play_to(size_t end)
{
    {
        size_t op_count = operation_count();
        if (end >= op_count) {
            end = op_count;
        }
    }
    size_t& start = current_index;
    bool reverse_mode = start > end;
    std::vector<TriMeshOperationData> tri_ops;

    {
        // get a valid interval to select things with
        std::vector<size_t> start_vec, size_vec;
        if (reverse_mode) {
            start_vec.emplace_back(end);
            size_vec.emplace_back(start - end);
        } else {
            start_vec.emplace_back(start);
            size_vec.emplace_back(end - start);
        }

        logger.operation_dataset.select(start_vec, size_vec).read(tri_ops);
    }

    if (debug_play_with_executor) {
        ExecutePass<TriMesh, ExecutionPolicy::kSeq> scheduler;


        auto run = [&](const TriMeshOperationData& tri_op) {
            std::string op_name = tri_op.name;
            const TriMeshTupleData& tup = tri_op.input_tuple;
            ;
            TriMesh::Tuple edge(tup.vertex_id, tup.local_edge_id, tup.triangle_id, mesh);
            // TODO: how does reverse mode get incorporated
            scheduler.edit_operation_maps[op_name](mesh, edge);

            if (tri_op.update_range_begin != tri_op.update_range_end) {
                std::vector<AttributeChanges> attr_changes;
                std::vector<size_t> attr_start, attr_size;
                attr_start.emplace_back(tri_op.update_range_begin);
                attr_size.emplace_back(tri_op.update_range_end - tri_op.update_range_begin);

                logger.attribute_changes_dataset.select(attr_start, attr_size).read(attr_changes);
            }
        };

        assert(reverse_mode == false);

        if (reverse_mode) {
            for (auto it = tri_ops.rbegin(); it != tri_ops.rend(); ++it) {
                run(*it);
            }
        } else {
            for (const auto& tri_op : tri_ops) {
                run(tri_op);
            }
        }
    } else {
        auto run = [&](const TriMeshOperationData& tri_op) {
            std::string op_name = tri_op.name;
            const TriMeshTupleData& tup = tri_op.input_tuple;
            TriMesh::Tuple edge(tup.vertex_id, tup.local_edge_id, tup.triangle_id, mesh);
            spdlog::info("OperationReplayer playing op {} on tup {}", op_name, edge.info());

            if (tri_op.update_range_begin != tri_op.update_range_end) {
                std::vector<AttributeChanges> attr_changes;
                std::vector<size_t> attr_start, attr_size;
                attr_start.emplace_back(tri_op.update_range_begin);
                attr_size.emplace_back(tri_op.update_range_end - tri_op.update_range_begin);

                logger.attribute_changes_dataset.select(attr_start, attr_size).read(attr_changes);

                for (const AttributeChanges& attr_change : attr_changes) {
                    const std::string attr_name = attr_change.name;
                    spdlog::info("Attribute {} changed and size is {}", attr_name, attr_change.attribute_size);


                    if (auto it = datasets.find(attr_name); it == datasets.end()) {
                        datasets[attr_name] = logger.file.getDataSet(attr_name);
                    }

                    const HighFive::DataSet& ds = datasets[attr_name];

                    const auto& ar = logger.attribute_recorders;
                    if (auto it = ar.find(attr_name); it != ar.end()) {
                        auto& recorder = *it->second;
                        recorder.load(attr_change, ds);
                    }
                }
            }
            mesh.build_vertex_connectivity(tri_op.vertex_size);
            mesh.current_tri_size = tri_op.triangle_size;
        };


        if (reverse_mode) {
            for (auto it = tri_ops.rbegin(); it != tri_ops.rend(); ++it) {
                run(*it);
            }
        } else {
            for (const auto& tri_op : tri_ops) {
                run(tri_op);
            }
        }
    }
    return (current_index = end);
}

