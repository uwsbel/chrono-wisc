#=============================================================================
# CMake configuration file for Chrono Worlds module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(CH_ENABLE_MODULE_SENSOR "Enable the Chrono Worlds module" OFF)

if(NOT CH_ENABLE_MODULE_WORLDS)
  return()
endif()


message(STATUS "\n==== Chrono Worlds module ====\n")




add_library(Chrono::worlds ALIAS Chrono_worlds)
target_link_libraries(Chrono_worlds PRIVATE Chrono_core ${LIBRARIES})
