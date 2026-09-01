cmake_minimum_required(VERSION 3.25)

project(aerial_touch LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8)
endif()

include(CTest)

find_package(yaml-cpp CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core highgui imgproc imgcodecs)

set(ORBBEC_SDK_ROOT "C:/Program Files/OrbbecSDK 2.9.3" CACHE PATH "Orbbec SDK installation root")
find_package(OrbbecSDK CONFIG REQUIRED PATHS "${ORBBEC_SDK_ROOT}/lib" NO_DEFAULT_PATH)

add_library(aerial_touch_core STATIC
    src/app_config.cpp
    src/depth_sampler.cpp
    src/plane.cpp
    src/keypad.cpp
    src/touch_state_machine.cpp
    src/hand_tracker.cpp
)

target_include_directories(aerial_touch_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(aerial_touch_core PUBLIC yaml-cpp::yaml-cpp)

add_library(aerial_touch_runtime STATIC
    src/orbbec_camera.cpp
)
target_include_directories(aerial_touch_runtime PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(aerial_touch_runtime PUBLIC aerial_touch_core ob::OrbbecSDK ${OpenCV_LIBS})

add_executable(aerial_touch_app
    src/main.cpp
    src/utf8_text.cpp
)
target_link_libraries(aerial_touch_app PRIVATE aerial_touch_runtime)
if(WIN32)
    target_link_libraries(aerial_touch_app PRIVATE gdi32 user32)
endif()

add_executable(orbbec_stream_probe tools/orbbec_stream_probe.cpp)
target_link_libraries(orbbec_stream_probe PRIVATE ob::OrbbecSDK)

add_custom_command(TARGET aerial_touch_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:aerial_touch_app>/config"
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:aerial_touch_app>/assets/models"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${ORBBEC_SDK_ROOT}/bin/extensions"
        "$<TARGET_FILE_DIR:aerial_touch_app>/extensions"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/config/default.yaml"
        "$<TARGET_FILE_DIR:aerial_touch_app>/config/default.yaml"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:ob::OrbbecSDK>"
        "$<TARGET_FILE_DIR:aerial_touch_app>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ORBBEC_SDK_ROOT}/bin/OrbbecSDKConfig.xml"
        "$<TARGET_FILE_DIR:aerial_touch_app>/OrbbecSDKConfig.xml"
)

add_custom_command(TARGET orbbec_stream_probe POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${ORBBEC_SDK_ROOT}/bin/extensions"
        "$<TARGET_FILE_DIR:orbbec_stream_probe>/extensions"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:ob::OrbbecSDK>"
        "$<TARGET_FILE_DIR:orbbec_stream_probe>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ORBBEC_SDK_ROOT}/bin/OrbbecSDKConfig.xml"
        "$<TARGET_FILE_DIR:orbbec_stream_probe>/OrbbecSDKConfig.xml"
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/models/hand_landmarker.task")
    add_custom_command(TARGET aerial_touch_app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/assets/models/hand_landmarker.task"
            "$<TARGET_FILE_DIR:aerial_touch_app>/assets/models/hand_landmarker.task"
    )
endif()

if(BUILD_TESTING)
    add_test(NAME aerial_touch_localization
        COMMAND ${CMAKE_COMMAND}
            "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_localization.cmake)
    add_test(NAME aerial_touch_build_script
        COMMAND ${CMAKE_COMMAND}
            "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_build_script.cmake)
    add_test(NAME aerial_touch_release_build
        COMMAND ${CMAKE_COMMAND}
            "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_release_build.cmake)
    add_test(NAME aerial_touch_orbbec_probe_config
        COMMAND ${CMAKE_COMMAND}
            "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_orbbec_probe_config.cmake)

    add_executable(aerial_touch_tests
        tests/test_depth_sampler.cpp
        tests/test_interaction_core.cpp
    )

    target_link_libraries(aerial_touch_tests PRIVATE aerial_touch_core)
    target_compile_definitions(aerial_touch_tests PRIVATE TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME aerial_touch_tests COMMAND aerial_touch_tests)

    add_executable(aerial_touch_bridge_smoke tests/test_bridge_smoke.cpp)
    target_link_libraries(aerial_touch_bridge_smoke PRIVATE aerial_touch_core ${OpenCV_LIBS})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/models/hand_landmarker.task"
       AND EXISTS "${CMAKE_BINARY_DIR}/mediapipe_hand_bridge.dll")
        add_test(NAME aerial_touch_bridge_smoke
            COMMAND aerial_touch_bridge_smoke
                "${CMAKE_BINARY_DIR}/mediapipe_hand_bridge.dll"
                "${CMAKE_CURRENT_SOURCE_DIR}/assets/models/hand_landmarker.task"
                "${CMAKE_CURRENT_SOURCE_DIR}/tests/data/right_hands.bmp")
    endif()
endif()
