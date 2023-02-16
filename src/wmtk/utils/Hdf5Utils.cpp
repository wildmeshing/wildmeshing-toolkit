#include <wmtk/utils/Hdf5Utils.h>
#include <wmtk/utils/Logger.hpp>
namespace wmtk {
// checks whether the file holding the dataset exists
bool does_dataset_exist(const HighFive::File& file, const std::string& name)
{
    auto obj_names = file.listObjectNames();
    if (file.exist(name)) {
        return true;
        ;
    }
    // for whaterver reason file.exist doesn't work with my invocation so doing it the dumb way
    for (auto&& n : obj_names) {
        if (n == name) {
            if (HighFive::ObjectType::Dataset == file.getObjectType(name)) {
                return true;
            } else {
                logger().error(
                    "create_dataset: {} had root node {} but it was not a dataset",
                    file.getName(),
                    name);
            }
        }
    }
    return false;
}
HighFive::DataSet
create_dataset(HighFive::File& file, const std::string& name, const HighFive::DataType& datatype)
{
    if (does_dataset_exist(file, name)) {
        auto ds = file.getDataSet(name);
        return ds;
    } else {
        HighFive::DataSetCreateProps props;
        props.add(HighFive::Chunking(std::vector<hsize_t>{2}));
        return file.createDataSet(
            std::string(name),
            // create an empty dataspace of unlimited size
            HighFive::DataSpace({0}, {HighFive::DataSpace::UNLIMITED}),
            // configure its datatype according to derived class's datatype spec
            datatype,
            // should enable chunking to allow appending
            props);
    }
}

} // namespace wmtk
