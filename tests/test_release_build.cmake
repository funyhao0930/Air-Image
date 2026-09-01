if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

set(RELEASE_FILES
    "${SOURCE_DIR}/CMakePresets.json"
    "${SOURCE_DIR}/README.md"
    "${SOURCE_DIR}/USER_GUIDE.zh-TW.md"
    "${SOURCE_DIR}/scripts/build_mediapipe_bridge.ps1")

foreach(FILE_PATH IN LISTS RELEASE_FILES)
    file(READ "${FILE_PATH}" FILE_TEXT)
    string(FIND "${FILE_TEXT}" "windows-debug" DEBUG_REFERENCE_INDEX)
    if(NOT DEBUG_REFERENCE_INDEX EQUAL -1)
        message(FATAL_ERROR "仍有 windows-debug 參照：${FILE_PATH}")
    endif()
endforeach()

file(READ "${SOURCE_DIR}/CMakePresets.json" PRESETS_TEXT)
foreach(REQUIRED_TEXT IN ITEMS
        "\"name\": \"windows-release\""
        "\"binaryDir\": \"\${sourceDir}/build/windows-release\""
        "\"CMAKE_BUILD_TYPE\": \"Release\"")
    string(FIND "${PRESETS_TEXT}" "${REQUIRED_TEXT}" MATCH_INDEX)
    if(MATCH_INDEX EQUAL -1)
        message(FATAL_ERROR "缺少 Release preset 設定：${REQUIRED_TEXT}")
    endif()
endforeach()

file(READ "${SOURCE_DIR}/scripts/build_mediapipe_bridge.ps1" SCRIPT_TEXT)
string(FIND "${SCRIPT_TEXT}" "[string]$OutputDirectory = \"build/windows-release\"" RELEASE_DEFAULT_INDEX)
if(RELEASE_DEFAULT_INDEX EQUAL -1)
    message(FATAL_ERROR "MediaPipe bridge 預設輸出目錄不是 build/windows-release")
endif()
