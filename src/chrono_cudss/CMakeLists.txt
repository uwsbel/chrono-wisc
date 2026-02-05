#=============================================================================
# CMake configuration file for the Chrono cuDSS module
#
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(CH_ENABLE_MODULE_CUDSS "Enable the Chrono cuDSS module" OFF)

if(NOT CH_ENABLE_MODULE_CUDSS)
    return()
endif()

message(STATUS "\n==== Chrono cuDSS module ====\n")

# ------------------------------------------------------------------------------
# Check for CUDA availability
# ------------------------------------------------------------------------------

if(NOT CHRONO_CUDA_FOUND)
    message(WARNING "CUDA not found. Disabling cuDSS module.")
    set(CH_ENABLE_MODULE_CUDSS OFF CACHE BOOL "Enable the Chrono cuDSS module" FORCE)
    return()
endif()

# ------------------------------------------------------------------------------
# Find cuDSS library
# ------------------------------------------------------------------------------

message(STATUS "Find cuDSS library")

# User can set CUDSS_DIR to specify cuDSS installation location
set(CUDSS_DIR "" CACHE PATH "Path to cuDSS installation directory")

# Find cuDSS library
find_library(CUDSS_LIBRARY
    NAMES cudss
    HINTS
        ${CUDAToolkit_LIBRARY_DIR}
        ${CUDSS_DIR}/lib
        ${CUDSS_DIR}/lib64
    DOC "cuDSS library"
)

# Find cuDSS header
find_path(CUDSS_INCLUDE_DIR
    NAMES cudss.h
    HINTS
        ${CUDAToolkit_INCLUDE_DIRS}
        ${CUDSS_DIR}/include
    DOC "cuDSS include directory"
)

if(NOT CUDSS_LIBRARY OR NOT CUDSS_INCLUDE_DIR)
    message(WARNING "cuDSS not found. Disabling cuDSS module.")
    set(CH_ENABLE_MODULE_CUDSS OFF CACHE BOOL "Enable the Chrono cuDSS module" FORCE)
    return()
endif()

message(STATUS "  cuDSS library:     ${CUDSS_LIBRARY}")
message(STATUS "  cuDSS include dir: ${CUDSS_INCLUDE_DIR}")

# ------------------------------------------------------------------------------
# List all files in the Chrono cuDSS module
# ------------------------------------------------------------------------------

set(Chrono_CuDSS_HEADERS
  ChApiCuDSS.h
  ChSolverCuDSS.h
)

set(Chrono_CuDSS_SOURCES
  ChSolverCuDSS.cu
)

source_group("" FILES ${Chrono_CuDSS_HEADERS} ${Chrono_CuDSS_SOURCES})

# ------------------------------------------------------------------------------
# Add the Chrono_cudss library
# ------------------------------------------------------------------------------

add_library(Chrono_cudss
            ${Chrono_CuDSS_SOURCES}
            ${Chrono_CuDSS_HEADERS})
add_library(Chrono::cudss ALIAS Chrono_cudss)

set_target_properties(Chrono_cudss PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_cudss PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_cudss PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_cudss PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_cudss PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

# Set CUDA architecture
if(CHRONO_CUDA_ARCHITECTURES)
  set_target_properties(Chrono_cudss PROPERTIES CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES})
endif()

target_link_libraries(Chrono_cudss
                      Chrono_core
                      CUDA::cudart
                      ${CUDSS_LIBRARY}
                      )

target_include_directories(Chrono_cudss PRIVATE ${CUDSS_INCLUDE_DIR})

target_compile_definitions(Chrono_cudss PRIVATE "CH_API_COMPILE_CUDSS")

install(TARGETS Chrono_cudss
  EXPORT ChronoTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include/chrono_cudss)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

install(FILES ${Chrono_CuDSS_HEADERS}
        DESTINATION include/chrono_cudss)
