#=============================================================================
# CMake configuration file for Chrono Worlds module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(CH_ENABLE_MODULE_WORLDS "Enable the Chrono Worlds module" OFF)

if(NOT CH_ENABLE_MODULE_WORLDS)
  return()
endif()


message(STATUS "\n==== Chrono Worlds module ====\n")

set(Chrono_worlds_QUADTREE_SOURCES
  quadtree/QuadtreeWorld.cpp
  quadtree/QuadtreeTile.cpp
  quadtree/GeoTIFFLoader.cpp
  quadtree/Heightfield.cpp
)

set(Chrono_worlds_QUADTREE_HEADERS
  quadtree/Quadtree.h
  quadtree/QuadtreeWorld.h
  quadtree/QuadtreeTile.h
  quadtree/GeoTIFFLoader.h
  quadtree/Heightfield.h
  quadtree/Perlin.h
)

set(Chrono_worlds_SOURCES
  ChWorldTerrainManager.cpp
  ChWorldTerrainVisualizer.cpp
)

set(Chrono_worlds_HEADERS
  ChApiWorlds.h
  ChWorldTerrainManager.h
  ChWorldTerrainVisualizer.h
)

source_group("Source" FILES
    ${Chrono_worlds_SOURCES}
  	${Chrono_worlds_HEADERS}
)

source_group("Quadtree" FILES
    ${Chrono_worlds_QUADTREE_SOURCES}
  	${Chrono_worlds_QUADTREE_HEADERS}
)


list(APPEND ALL_CH_WORLDS_FILES ${Chrono_worlds_SOURCES})
list(APPEND ALL_CH_WORLDS_FILES ${Chrono_worlds_HEADERS})
list(APPEND ALL_CH_WORLDS_FILES ${Chrono_worlds_QUADTREE_SOURCES})
list(APPEND ALL_CH_WORLDS_FILES ${Chrono_worlds_QUADTREE_HEADERS})

add_library(Chrono_worlds ${ALL_CH_WORLDS_FILES})
add_library(Chrono::worlds ALIAS Chrono_worlds)

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_worlds PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_worlds PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_worlds PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_worlds PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:NOMINMAX>)
endif()

#################
# Find the GDAL package.
# This will look for the GDALConfig.cmake file or use the FindGDAL module.
# add cmake prefix path to search for the GDALConfig.cmake file
list(APPEND CMAKE_PREFIX_PATH "C:\\OSGeo4W" "C:\\Packages")
find_package(GDAL REQUIRED)

if(NOT GDAL_FOUND)
    message(FATAL_ERROR "GDAL not found. Please install GDAL and try again.")
endif()

message(STATUS "Found GDAL version: ${GDAL_VERSION_STRING}")
message(STATUS "GDAL include dir: ${GDAL_INCLUDE_DIRS}")
message(STATUS "GDAL libraries: ${GDAL_LIBRARIES}")

target_include_directories(Chrono_worlds PUBLIC ${GDAL_INCLUDE_DIRS})
target_link_libraries(Chrono_worlds PUBLIC ${GDAL_LIBRARIES})

###############
find_package(OpenGL QUIET)
find_package(GLEW QUIET)
find_package(glfw3 QUIET)

message(STATUS "OpenGL found: ${OPENGL_FOUND}")
message(STATUS "   include dir: ${OPENGL_INCLUDE_DIR}")
message(STATUS "   libraries:   ${OPENGL_LIBRARIES}")
message(STATUS "GLEW found:   ${GLEW_FOUND}")
message(STATUS "   include dir: ${GLEW_INCLUDE_DIRS}")
message(STATUS "   libraries:   ${GLEW_LIBRARIES}")
message(STATUS "glfw3 found:  ${glfw3_FOUND}")

if(OPENGL_FOUND AND GLEW_FOUND AND glfw3_FOUND)
  # target_compile_definitions(Chrono_worlds PUBLIC $<$<COMPILE_LANGUAGE:CXX>:USE_SENSOR_GLFW>)
  target_link_libraries(Chrono_worlds PUBLIC GLEW::glew)
  target_link_libraries(Chrono_worlds PUBLIC glfw)
  target_link_libraries(Chrono_worlds PUBLIC OpenGL::GL)
  message(STATUS "GL libraries found")
else()
  message(STATUS "GL libraries not found; OpenGL support disabled")
endif()


### Add GLAD
add_library(glad ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/glad/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/glad/include)
target_link_libraries(Chrono_worlds PUBLIC glad)

################

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigWorlds.h.in
    ${PROJECT_BINARY_DIR}/chrono_worlds/ChConfigWorlds.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_worlds/ChConfigWorlds.h"
    DESTINATION include/chrono_worlds)


# windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_options(Chrono_worlds PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4661>)
    target_compile_options(Chrono_worlds PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4005>)
endif()

target_compile_definitions(Chrono_worlds PUBLIC $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_WORLDS>)
target_link_libraries(Chrono_worlds PRIVATE Chrono_core ${LIBRARIES})


#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------


install(TARGETS Chrono_worlds
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_worlds)


# Old way
install(FILES ${Chrono_worlds_HEADERS}
		DESTINATION include/chrono_worlds)
install(FILES ${Chrono_worlds_QUADTREE_HEADERS}
    DESTINATION include/chrono_worlds/quadtree)

# On Windows, extract DLLs from targets and install (always look for Release DLLs)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    get_target_property(GLEW_DLL GLEW::glew IMPORTED_LOCATION_RELEASE)
    get_target_property(GLFW_DLL glfw IMPORTED_LOCATION_RELEASE)
    if(EXISTS "${GLEW_DLL}")
      install(FILES "${GLEW_DLL}" DESTINATION bin)
    endif()
    if(EXISTS "${GLFW_DLL}")
      install(FILES "${GLFW_DLL}" DESTINATION bin)
    endif()
endif()
