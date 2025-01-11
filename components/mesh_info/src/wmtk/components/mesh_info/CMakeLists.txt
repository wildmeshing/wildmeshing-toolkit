add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()
set(SRC_FILES
    mesh_info.cpp
    mesh_info.hpp
    element_count_report.hpp
    element_count_report.cpp

    simplex/volumes.hpp
    simplex/volumes.cpp
    simplex/operators/VolumeOperator.hpp

    simplex/dihedral_angles.hpp
    simplex/dihedral_angles.cpp
    simplex/operators/DihedralAngleOperator.hpp
)


target_compile_features(wmtk_${COMPONENT_NAME} PUBLIC cxx_std_20)


target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE nlohmann_json)
target_link_libraries(wmtk_${COMPONENT_NAME} PUBLIC wmtk::multimesh)

