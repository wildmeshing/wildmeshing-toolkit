if (TARGET CLI11::CLI11)
    return()
endif()

message(STATUS "Third-party (external): creating target 'CLI11::CLI11'")

include(CPM)
CPMAddPackage("gh:CLIUtils/CLI11@2.4.2")

set_target_properties(CLI11 PROPERTIES FOLDER third_party)