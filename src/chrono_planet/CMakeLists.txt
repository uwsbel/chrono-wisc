#=============================================================================
# CMake configuration file for the Chrono Planet module
#
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(CH_ENABLE_MODULE_PLANET "Enable the Chrono Planet module (GeoTIFF DEM terrain with quadtree LOD)" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_PLANET)
  return()
endif()

message(STATUS "\n==== Chrono Planet module ====\n")

#-----------------------------------------------------------------------------
# GDAL reads the GeoTIFFs. CMake's own FindGDAL provides the GDAL::GDAL target.
#-----------------------------------------------------------------------------

find_package(GDAL REQUIRED)
message(STATUS "GDAL version: ${GDAL_VERSION}")

#-----------------------------------------------------------------------------
# ISA for the terrain core's grid loops. Contraction stays off so every build
# sums the same bits; "native" is faster but not reproducible across machines.
#-----------------------------------------------------------------------------

set(CH_PLANET_SIMD "avx2" CACHE STRING "Planet module grid-loop ISA: avx2, native or off")
set_property(CACHE CH_PLANET_SIMD PROPERTY STRINGS avx2 native off)
if(CH_PLANET_SIMD STREQUAL "native")
  set(CH_PLANET_SIMD_FLAGS "$<$<CXX_COMPILER_ID:GNU,Clang>:-march=native;-ffp-contract=off>")
elseif(CH_PLANET_SIMD STREQUAL "avx2")
  set(CH_PLANET_SIMD_FLAGS "$<$<CXX_COMPILER_ID:GNU,Clang>:-mavx2;-ffp-contract=off>")
else()
  set(CH_PLANET_SIMD_FLAGS "$<$<CXX_COMPILER_ID:GNU,Clang>:-ffp-contract=off>")
endif()

#-----------------------------------------------------------------------------

set(Chrono_PLANET_FILES
    ChApiPlanet.h
    ChGeoGrid.h
    ChLevelOfDetail.h
    ChLevelOfDetail.cpp
    ChPlanetBody.h
    ChPlanetBody.cpp
    ChPlanetSurface.h
    ChPlanetSurface.cpp
    ChPlanetVisualMesh.h
    ChPlanetVisualMesh.cpp
    ChSiteFrame.h
    ChSiteHoles.h
    ChSiteHoles.cpp
)
source_group("" FILES ${Chrono_PLANET_FILES})

set(Chrono_PLANET_CORE_FILES
    core/BenchProfiler.h
    core/BenchProfiler.cpp
    core/FieldHash.h
    core/MathUtil.h
    core/MathUtil.cpp
    core/Parallel.h
    core/Perlin.h
    core/Perlin.cpp
    core/SphereMath.h
    core/SphereMath.cpp
)
source_group("core" FILES ${Chrono_PLANET_CORE_FILES})

set(Chrono_PLANET_SAMPLERS_FILES
    samplers/ChElevationSampler.h
    samplers/ChElevationSampler.cpp
)
source_group("samplers" FILES ${Chrono_PLANET_SAMPLERS_FILES})

set(Chrono_PLANET_DEM_FILES
    samplers/ChElevationSampler.h
    samplers/ChElevationSampler.cpp
    dem/ChGeoTiffStack.h
    dem/ChGeoTiffStack.cpp
    dem/GeoTIFFLoader.h
    dem/GeoTIFFLoader.cpp
)
source_group("dem" FILES ${Chrono_PLANET_DEM_FILES})

set(Chrono_PLANET_LOD_FILES
    lod/ChPlanetQuadtree.h
    lod/ChPlanetQuadtree.cpp
    lod/ChTileMesh.h
    lod/CoordinateSystems.h
    lod/Quadtree.h
    lod/Quadtree.cpp
    lod/QuadtreeTile.h
    lod/QuadtreeTile.cpp
    lod/SphericalCoordinates.h
    lod/CartesianCoordinates.h
    lod/CartesianCoordinates.cpp
    lod/SphericalCoordinates.cpp
    lod/TileMeshBuilder.h
    lod/TileMeshBuilder.cpp
    lod/TileMetadata.h
    lod/TileMetadata.cpp
)
source_group("lod" FILES ${Chrono_PLANET_LOD_FILES})

set(Chrono_PLANET_FILTERS_FILES
    filters/ChSurfaceFilter.h
    filters/ChSurfaceFilter.cpp
    filters/ChDeformationFilter.h
    filters/ChDeformationFilter.cpp
)
source_group("filters" FILES ${Chrono_PLANET_FILTERS_FILES})

set(Chrono_PLANET_GEOMETRY_FILES
    geometry/ChSdfMesher.h
    geometry/ChSdfMesher.cpp
)
source_group("geometry" FILES ${Chrono_PLANET_GEOMETRY_FILES})

set(Chrono_PLANET_PROCEDURAL_FILES
    procedural/ChReliefLayer.h
    procedural/ChReliefLayer.cpp
    procedural/ChBaseReliefLayer.h
    procedural/ChBaseReliefLayer.cpp
    procedural/ChBedformLayer.h
    procedural/ChBedformLayer.cpp
    procedural/ChExposureField.h
    procedural/ChExposureField.cpp
    procedural/ChCraterLayer.h
    procedural/ChCraterLayer.cpp
    procedural/ChRockLayer.h
    procedural/ChRockLayer.cpp
    procedural/ChRoughnessLayer.h
    procedural/ChRoughnessLayer.cpp
    procedural/ChIsotropicRelief.h
    procedural/ChIsotropicRelief.cpp
    procedural/FieldGrid.h
    core/FieldHash.h
    core/Perlin.h
    core/Perlin.cpp
    geometry/RockMeshes.h
    geometry/RockMeshes.cpp
    geometry/RockSdf.h
    geometry/RockSdf.cpp
)
source_group("procedural" FILES ${Chrono_PLANET_PROCEDURAL_FILES})


# Presets for particular bodies. They use only the public API above; nothing in the
# core includes them.
set(Chrono_PLANET_PLANETS_FILES
    planets/moon/ChMoon.h
    planets/moon/ChMoon.cpp
    planets/moon/ChMoonDem.h
    planets/moon/ChMoonDem.cpp
    planets/mars/ChMars.h
    planets/mars/ChMars.cpp
)
source_group("planets" FILES ${Chrono_PLANET_PLANETS_FILES})

# The grid loops in these files only vectorise if the compiler may hoist
# floating-point work out of a conditional and skip the errno check on sqrt.
# Neither changes a result and nothing here reads errno.
set_source_files_properties(
    procedural/ChBaseReliefLayer.cpp
    procedural/ChBedformLayer.cpp
    procedural/ChCraterLayer.cpp
    procedural/ChRockLayer.cpp
    procedural/ChRoughnessLayer.cpp
    dem/ChGeoTiffStack.cpp
    lod/SphericalCoordinates.cpp
    lod/TileMeshBuilder.cpp
    PROPERTIES COMPILE_OPTIONS "$<$<CXX_COMPILER_ID:GNU,Clang>:-fno-math-errno;-fno-trapping-math>"
)

#-----------------------------------------------------------------------------
# In most cases, you do not need to edit the lines below.

add_library(Chrono_planet
    ${Chrono_PLANET_FILES}
    ${Chrono_PLANET_CORE_FILES}
    ${Chrono_PLANET_SAMPLERS_FILES}
    ${Chrono_PLANET_DEM_FILES}
    ${Chrono_PLANET_LOD_FILES}
    ${Chrono_PLANET_FILTERS_FILES}
    ${Chrono_PLANET_GEOMETRY_FILES}
    ${Chrono_PLANET_PROCEDURAL_FILES}
    ${Chrono_PLANET_GEOMETRY_FILES}
    ${Chrono_PLANET_PLANETS_FILES}
)
add_library(Chrono::planet ALIAS Chrono_planet)

set_target_properties(Chrono_planet PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_planet PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_planet PROPERTIES LINK_FLAGS "/LTCG")
endif()

if(CH_STATIC)
  set_target_properties(Chrono_planet PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_planet PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_planet PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_PLANET>)

target_compile_options(Chrono_planet PRIVATE ${CH_PLANET_SIMD_FLAGS})

# GDAL stays inside the DEM implementation; installed headers do not include it.
# Users link GDAL but need not have its headers.
target_link_libraries(Chrono_planet PUBLIC Chrono_core PRIVATE GDAL::GDAL)

install(TARGETS Chrono_planet
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_planet)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_planet
        FILES_MATCHING PATTERN "*.h"
        PATTERN "GeoTIFFLoader.h" EXCLUDE)

# Dependencies on other Chrono components, for the exported ChronoConfig.
set(DEPENDENCIES_PLANET "" PARENT_SCOPE)

#-------------------------------------------------------------------------------
# Add the Chrono_planet_vsg library: the quadtree terrain as a VSG plugin
#-------------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VSG)
  message(STATUS "Add Chrono_planet_vsg library")

  set(Chrono_PLANET_VSG_FILES
      visualization/ChPlanetVisualizationVSG.h
      visualization/ChPlanetVisualizationVSG.cpp
      visualization/ChMeshVisualizationVSG.h
      visualization/ChMeshVisualizationVSG.cpp
      visualization/ChMeshDrawsVSG.h
      visualization/ChMeshDrawsVSG.cpp
  )
  source_group("visualization" FILES ${Chrono_PLANET_VSG_FILES})

  add_library(Chrono_planet_vsg ${Chrono_PLANET_VSG_FILES})
  add_library(Chrono::planet_vsg ALIAS Chrono_planet_vsg)

  set_target_properties(Chrono_planet_vsg PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

  if(CH_WHOLE_PROG_OPT)
    set_target_properties(Chrono_planet_vsg PROPERTIES COMPILE_FLAGS "/GL")
    set_target_properties(Chrono_planet_vsg PROPERTIES LINK_FLAGS "/LTCG")
  endif()

  if(CH_STATIC)
    set_target_properties(Chrono_planet_vsg PROPERTIES POSITION_INDEPENDENT_CODE ON)
  endif()

  if(MSVC)
    set_target_properties(Chrono_planet_vsg PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()

  target_compile_definitions(Chrono_planet_vsg PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_PLANET>)

  target_link_libraries(Chrono_planet_vsg PUBLIC Chrono_planet Chrono_vsg)

  install(TARGETS Chrono_planet_vsg
          EXPORT ChronoTargets
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          INCLUDES DESTINATION include/chrono_planet)
endif()
