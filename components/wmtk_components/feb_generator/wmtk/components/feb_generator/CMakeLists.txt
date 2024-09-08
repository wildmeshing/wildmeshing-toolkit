
set(SRC_FILES
    internal/FEBGenerator.hpp
    internal/FEBGenerator.cpp
    internal/FEBContactUtils.hpp
    internal/FEBGeneratorOptions.hpp
    feb_generator.cpp
	feb_generator.hpp)


include(libigl)

target_sources(${CURRENT_COMPONENT_LIB_NAME} PRIVATE ${SRC_FILES})
target_link_libraries(${CURRENT_COMPONENT_LIB_NAME} PRIVATE igl::core)
