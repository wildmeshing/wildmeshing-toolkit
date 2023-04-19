
#include <spdlog/spdlog.h>
#include <wmtk/serialization/AttributeCollectionSerializer.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/serialization/Hdf5Utils.hpp>


using namespace wmtk;
AttributeCollectionSerializerBase::AttributeCollectionSerializerBase(
    HighFive::File& file,
    const std::string& name,
    const HighFive::DataType& data_type)
{

    if(utils::does_dataset_exist(file, name)) {
        m_dataset = file.getDataSet(name);
        assert(m_dataset == data_type);
    } else {
        m_dataset = utils::create_extendable_dataset(file, name, data_type);
        spdlog::warn("{}", m_dataset.getMemSpace().getNumberDimensions());
    }
}

AttributeCollectionSerializerBase::~AttributeCollectionSerializerBase() {}
