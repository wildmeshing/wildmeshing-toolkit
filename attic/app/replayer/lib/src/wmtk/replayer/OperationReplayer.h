#pragma once

#include <wmtk/utils/Hdf5Utils.h>
#include <map>

namespace wmtk {
class TriMesh;

class OperationLogger;

class OperationReplayer
{
public:
    OperationReplayer(TriMesh& mesh, const OperationLogger& logger);

    // test replay by actually executing operations
    bool debug_play_with_executor = false;

    size_t play(int step_count);
    size_t play_to(size_t end);
    size_t operation_count() const;

private:
    // start must be current_index
    TriMesh& mesh;
    const OperationLogger& logger;
    std::map<std::string, HighFive::DataSet> datasets;
    size_t current_index = 0;
    // HighFive::File& file;
    // HighFive::DataSet operation_dataset;
    // HighFive::DataSet attribute_changes_dataset;
    //// std::ostream& output_stream;
    // std::map<std::string, AttributeCollectionRecorderBase*> attribute_recorders;


    //// returns true if attribute was successfully recorded
    // std::array<size_t, 2> record_attribute(const std::string& attribute_name);
};

} // namespace wmtk
