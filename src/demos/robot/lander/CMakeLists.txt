#=============================================================================
# CMake configuration file for the ROBOT lander demos.
#=============================================================================

if(NOT CH_ENABLE_MODULE_VEHICLE)
    return()
endif()

#--------------------------------------------------------------
# List model files shared by the lander demos

set(MODEL_FILES
    model/Lander.cpp
    model/Lander.h
)

set(LANDER_POSTPROCESS_SOURCES "")
set(LANDER_POSTPROCESS_LIBS "")

if(CH_ENABLE_MODULE_POSTPROCESS)
    list(APPEND LANDER_POSTPROCESS_LIBS Chrono_postprocess)
else()
    list(APPEND LANDER_POSTPROCESS_SOURCES ${CMAKE_SOURCE_DIR}/src/chrono_postprocess/ChBlender.cpp)
endif()

set(LANDER_PROGRAM demo_ROBOT_Lander)
set(LANDER_CRM_PROGRAM demo_ROBOT_Lander_CRM)
set(LANDER_CRM_FLAT_PROGRAM demo_ROBOT_Lander_CRM_Flat)
set(LANDER_CRM_BENCHMARK_PROGRAM demo_ROBOT_Lander_CRM_RTFBenchmark)

source_group("model" FILES ${MODEL_FILES})
source_group("postprocess" FILES ${LANDER_POSTPROCESS_SOURCES})

#--------------------------------------------------------------
# Common include directories

set(LANDER_INCLUDES
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/model
)

#--------------------------------------------------------------


if(CH_ENABLE_MODULE_VSG)
    add_executable(${LANDER_PROGRAM} ${LANDER_PROGRAM}.cpp ${MODEL_FILES})
    source_group("" FILES ${LANDER_PROGRAM}.cpp)

    target_include_directories(${LANDER_PROGRAM} PRIVATE ${LANDER_INCLUDES})
    target_link_libraries(${LANDER_PROGRAM}
        Chrono_core
        ChronoModels_robot
        Chrono_vehicle
        Chrono_vsg
        Chrono_vehicle_vsg
    )

    set_target_properties(${LANDER_PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
        set_target_properties(${LANDER_PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    if(MSVC)
        set_target_properties(${LANDER_PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    set_property(TARGET ${LANDER_PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${LANDER_PROGRAM}>")

    install(TARGETS ${LANDER_PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endif()

#--------------------------------------------------------------
# CRM lander demo (requires FSI + VSG)

if(CH_ENABLE_MODULE_FSI AND CH_ENABLE_MODULE_VSG)
    add_executable(${LANDER_CRM_PROGRAM} ${LANDER_CRM_PROGRAM}.cpp ${MODEL_FILES} ${LANDER_POSTPROCESS_SOURCES})
    source_group("" FILES ${LANDER_CRM_PROGRAM}.cpp)

    target_include_directories(${LANDER_CRM_PROGRAM} PRIVATE ${LANDER_INCLUDES})
    target_link_libraries(${LANDER_CRM_PROGRAM}
        Chrono_core
        ChronoModels_robot
        Chrono_vehicle
        Chrono_vsg
        Chrono_vehicle_vsg
        Chrono_fsisph
        Chrono_fsisph_vsg
        ${LANDER_POSTPROCESS_LIBS}
    )
    if(NOT CH_ENABLE_MODULE_POSTPROCESS)
        target_compile_definitions(${LANDER_CRM_PROGRAM} PRIVATE CH_API_COMPILE_POSTPROCESS)
    endif()

    set_target_properties(${LANDER_CRM_PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
        set_target_properties(${LANDER_CRM_PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    if(MSVC)
        set_target_properties(${LANDER_CRM_PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    set_property(TARGET ${LANDER_CRM_PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${LANDER_CRM_PROGRAM}>")

    install(TARGETS ${LANDER_CRM_PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})

    add_executable(${LANDER_CRM_FLAT_PROGRAM} ${LANDER_CRM_FLAT_PROGRAM}.cpp ${MODEL_FILES})
    source_group("" FILES ${LANDER_CRM_FLAT_PROGRAM}.cpp)

    target_include_directories(${LANDER_CRM_FLAT_PROGRAM} PRIVATE ${LANDER_INCLUDES})
    target_link_libraries(${LANDER_CRM_FLAT_PROGRAM}
        Chrono_core
        ChronoModels_robot
        Chrono_vehicle
        Chrono_vsg
        Chrono_vehicle_vsg
        Chrono_fsisph
        Chrono_fsisph_vsg
    )

    set_target_properties(${LANDER_CRM_FLAT_PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
        set_target_properties(${LANDER_CRM_FLAT_PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    if(MSVC)
        set_target_properties(${LANDER_CRM_FLAT_PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    set_property(TARGET ${LANDER_CRM_FLAT_PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY
                 "$<TARGET_FILE_DIR:${LANDER_CRM_FLAT_PROGRAM}>")

    install(TARGETS ${LANDER_CRM_FLAT_PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})

    add_executable(${LANDER_CRM_BENCHMARK_PROGRAM} ${LANDER_CRM_BENCHMARK_PROGRAM}.cpp ${MODEL_FILES}
                   ${LANDER_POSTPROCESS_SOURCES})
    source_group("" FILES ${LANDER_CRM_BENCHMARK_PROGRAM}.cpp)

    target_include_directories(${LANDER_CRM_BENCHMARK_PROGRAM} PRIVATE ${LANDER_INCLUDES})
    target_link_libraries(${LANDER_CRM_BENCHMARK_PROGRAM}
        Chrono_core
        ChronoModels_robot
        Chrono_vehicle
        Chrono_vsg
        Chrono_vehicle_vsg
        Chrono_fsisph
        Chrono_fsisph_vsg
        ${LANDER_POSTPROCESS_LIBS}
    )
    if(NOT CH_ENABLE_MODULE_POSTPROCESS)
        target_compile_definitions(${LANDER_CRM_BENCHMARK_PROGRAM} PRIVATE CH_API_COMPILE_POSTPROCESS)
    endif()

    set_target_properties(${LANDER_CRM_BENCHMARK_PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
        set_target_properties(${LANDER_CRM_BENCHMARK_PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    if(MSVC)
        set_target_properties(${LANDER_CRM_BENCHMARK_PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    set_property(TARGET ${LANDER_CRM_BENCHMARK_PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY
                 "$<TARGET_FILE_DIR:${LANDER_CRM_BENCHMARK_PROGRAM}>")

    install(TARGETS ${LANDER_CRM_BENCHMARK_PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endif()
