add_component(${COMPONENT_NAME})
if(NOT ${WMTK_ENABLE_COMPONENT_${COMPONENT_NAME}})
    return()
endif()
set(SRC_FILES

    RGB.hpp
    RGB.cpp
    RGB1.hpp
    RGB1.cpp
    RGB2.hpp
    RGB2.cpp
invariants/TriMeshRGBSplitInvariant.cpp
invariants/TriMeshRGBSwapInvariant.cpp
invariants/TriMeshRGBSplitInvariant.hpp
invariants/TriMeshRGBSwapInvariant.hpp
operations/RGBEdgeSplit.hpp
operations/RGBEdgeSplit.cpp
operations/RGBEdgeSwap.hpp
operations/RGBEdgeSwap.cpp
)

include(stb)


target_compile_features(wmtk_${COMPONENT_NAME} PUBLIC cxx_std_20)

target_sources(wmtk_${COMPONENT_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(wmtk_${COMPONENT_NAME} PRIVATE stb::image nlohmann_json)
